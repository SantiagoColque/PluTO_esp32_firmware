/*
 * Captive-portal DNS responder.
 *
 * The packet parsing and reply construction are adapted from the dns_server
 * component of the ESP-IDF captive_portal example:
 *   $IDF_PATH/examples/protocols/http_server/captive_portal/components/dns_server
 *   SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *   SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Three things are deliberately different from the original:
 *
 *  1. The original stop_dns_server() calls vTaskDelete() on a task blocked in
 *     recvfrom(), so the UDP socket is never closed and the handle is freed
 *     while the task may still be reading it. Because PluTO raises and drops
 *     the access point repeatedly, that leaks a socket per cycle until the
 *     device runs out. Here the task polls with a receive timeout, closes its
 *     own socket, and signals a semaphore that dns_hijack_stop() waits on.
 *  2. The socket binds to the access point address rather than INADDR_ANY, so
 *     the device never answers DNS for hosts on the real LAN while it is in
 *     AP+STA fallback.
 *  3. Per-packet logging is at DEBUG. A single phone probing for a captive
 *     portal emits hundreds of queries a second and would bury every other log
 *     line at INFO.
 */

#include "dns_hijack.h"

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"

#define DNS_PORT 53
#define DNS_MAX_LEN 256
#define DNS_RX_BUFFER_LEN 128
#define DNS_NAME_MAX_LEN 128
#define DNS_RECV_TIMEOUT_MS 500
#define DNS_STOP_TIMEOUT_MS 2000
#define DNS_TASK_STACK 3072
#define DNS_TASK_PRIORITY 5

#define DNS_OPCODE_MASK 0x7800
#define DNS_QR_FLAG (1 << 7)
#define DNS_TYPE_A 0x0001
#define DNS_ANSWER_TTL_SEC 300

static const char *TAG = "dns_hijack";

typedef struct __attribute__((__packed__))
{
    uint16_t id;
    uint16_t flags;
    uint16_t qd_count;
    uint16_t an_count;
    uint16_t ns_count;
    uint16_t ar_count;
} dns_header_t;

typedef struct {
    uint16_t type;
    uint16_t class;
} dns_question_t;

typedef struct __attribute__((__packed__))
{
    uint16_t ptr_offset;
    uint16_t type;
    uint16_t class;
    uint32_t ttl;
    uint16_t addr_len;
    uint32_t ip_addr;
} dns_answer_t;

struct dns_hijack_ctx {
    volatile bool running;
    uint32_t bind_addr;
    TaskHandle_t task;
    SemaphoreHandle_t finished;
};

/* Walk a length-prefixed DNS name, returning the first byte after it. */
static char *parse_dns_name(char *raw_name, char *parsed_name, size_t parsed_name_max_len)
{
    char *label = raw_name;
    char *name_itr = parsed_name;
    int name_len = 0;

    do {
        int sub_name_len = *label;

        name_len += (sub_name_len + 1);
        if (name_len > parsed_name_max_len) {
            return NULL;
        }

        memcpy(name_itr, label + 1, sub_name_len);
        name_itr[sub_name_len] = '.';
        name_itr += (sub_name_len + 1);
        label += sub_name_len + 1;
    } while (*label != 0);

    parsed_name[name_len - 1] = '\0';

    return label + 1;
}

static int build_dns_reply(char *req, size_t req_len, char *reply, size_t reply_max_len, uint32_t answer_ip)
{
    dns_header_t *header;
    uint16_t qd_count;
    uint16_t an_count = 0;
    int reply_len;
    char *cur_ans_ptr;
    char *cur_qd_ptr;
    char name[DNS_NAME_MAX_LEN];

    if (req_len > reply_max_len || req_len < sizeof(dns_header_t)) {
        return -1;
    }

    memset(reply, 0, reply_max_len);
    memcpy(reply, req, req_len);

    header = (dns_header_t *)reply;
    ESP_LOGD(TAG,
             "Query id=0x%X flags=0x%X qd_count=%d",
             ntohs(header->id),
             ntohs(header->flags),
             ntohs(header->qd_count));

    if ((header->flags & DNS_OPCODE_MASK) != 0) {
        return 0;
    }

    header->flags |= DNS_QR_FLAG;

    qd_count = ntohs(header->qd_count);

    if (req_len + qd_count * sizeof(dns_answer_t) > reply_max_len) {
        return -1;
    }

    cur_ans_ptr = reply + req_len;
    cur_qd_ptr = reply + sizeof(dns_header_t);

    for (int qd_i = 0; qd_i < qd_count; qd_i++) {
        char *name_end_ptr = parse_dns_name(cur_qd_ptr, name, sizeof(name));
        dns_question_t *question;
        dns_answer_t *answer;
        uint16_t qd_type;
        uint16_t qd_class;

        if (name_end_ptr == NULL) {
            ESP_LOGD(TAG, "Unable to parse the question name");
            return -1;
        }

        question = (dns_question_t *)name_end_ptr;
        qd_type = ntohs(question->type);
        qd_class = ntohs(question->class);

        if (qd_type == DNS_TYPE_A) {
            ESP_LOGD(TAG, "Answering A query for %s", name);

            answer = (dns_answer_t *)cur_ans_ptr;
            answer->ptr_offset = htons(0xC000 | (cur_qd_ptr - reply));
            answer->type = htons(qd_type);
            answer->class = htons(qd_class);
            answer->ttl = htonl(DNS_ANSWER_TTL_SEC);
            answer->addr_len = htons(sizeof(answer_ip));
            answer->ip_addr = answer_ip;

            cur_ans_ptr += sizeof(dns_answer_t);
            an_count++;
        }

        cur_qd_ptr = name_end_ptr + sizeof(dns_question_t);
    }

    /* Only count the answers actually written: a AAAA-only query must come back
     * with an_count 0 rather than one answer's worth of zeroes. */
    header->an_count = htons(an_count);
    reply_len = cur_ans_ptr - reply;

    return reply_len;
}

static void dns_hijack_task(void *arg)
{
    struct dns_hijack_ctx *ctx = arg;
    struct sockaddr_in bind_addr;
    struct timeval timeout;
    char rx_buffer[DNS_RX_BUFFER_LEN];
    char reply[DNS_MAX_LEN];
    int sock;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create the socket: errno %d", errno);
        goto done;
    }

    bind_addr.sin_addr.s_addr = ctx->bind_addr;
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(DNS_PORT);

    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "Unable to bind port %d: errno %d", DNS_PORT, errno);
        close(sock);
        goto done;
    }

    /* Without a receive timeout the task would sit in recvfrom() forever and
     * could only be stopped by deleting it, which is what leaks the socket. */
    timeout.tv_sec = 0;
    timeout.tv_usec = DNS_RECV_TIMEOUT_MS * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG, "Captive DNS answering on port %d", DNS_PORT);

    while (ctx->running) {
        struct sockaddr_in6 source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len;
        int reply_len;

        len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT) {
                continue;
            }
            ESP_LOGW(TAG, "recvfrom failed: errno %d", errno);
            break;
        }

        rx_buffer[len] = '\0';

        reply_len = build_dns_reply(rx_buffer, len, reply, sizeof(reply), ctx->bind_addr);
        if (reply_len <= 0) {
            continue;
        }

        if (sendto(sock, reply, reply_len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr)) < 0) {
            ESP_LOGD(TAG, "sendto failed: errno %d", errno);
        }
    }

    close(sock);
    ESP_LOGI(TAG, "Captive DNS stopped");

done:
    xSemaphoreGive(ctx->finished);
    vTaskDelete(NULL);
}

dns_hijack_handle_t dns_hijack_start(const char *netif_key)
{
    struct dns_hijack_ctx *ctx;
    esp_netif_t *netif;
    esp_netif_ip_info_t ip_info;

    if (netif_key == NULL) {
        return NULL;
    }

    netif = esp_netif_get_handle_from_ifkey(netif_key);
    if (netif == NULL) {
        ESP_LOGE(TAG, "No interface named %s", netif_key);
        return NULL;
    }

    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
        ESP_LOGE(TAG, "Interface %s has no address yet", netif_key);
        return NULL;
    }

    ctx = calloc(1, sizeof(*ctx));
    if (ctx == NULL) {
        return NULL;
    }

    ctx->finished = xSemaphoreCreateBinary();
    if (ctx->finished == NULL) {
        free(ctx);
        return NULL;
    }

    ctx->running = true;
    ctx->bind_addr = ip_info.ip.addr;

    if (xTaskCreate(dns_hijack_task, "dns_hijack", DNS_TASK_STACK, ctx, DNS_TASK_PRIORITY, &ctx->task) != pdPASS) {
        vSemaphoreDelete(ctx->finished);
        free(ctx);
        return NULL;
    }

    return ctx;
}

void dns_hijack_stop(dns_hijack_handle_t handle)
{
    if (handle == NULL) {
        return;
    }

    handle->running = false;

    /* Wait for the task to close its socket before freeing anything it reads. */
    if (xSemaphoreTake(handle->finished, pdMS_TO_TICKS(DNS_STOP_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "DNS task did not exit within %d ms; leaking its context", DNS_STOP_TIMEOUT_MS);
        return;
    }

    vSemaphoreDelete(handle->finished);
    free(handle);
}
