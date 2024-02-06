#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_https_server.h>
#include "esp_tls.h"
#include "sdkconfig.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
*/

static const char *TAG = "example";

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    // Sample sensor values (replace with actual sensor data)
    float sensorValue1 = 23.45;
    float sensorValue2 = 67.89;

    // Dynamic HTML content with placeholders for sensor values and styling
    const char* html_template = "<html>"
                                "<head>"
                                "<title>SMART GARDEN</title>"
                                "<style>"
                                "body { font-family: Arial, sans-serif; margin: 40px; background-color: #e0f7fa; }"
                                "h1 { color: #004d40; text-align: center; }"
                                "table { border-collapse: collapse; width: 50%; margin: 20px auto; background-color: #ffffff; }"
                                "th, td { border: 1px solid #ddd; padding: 10px; text-align: left; }"
                                "th { background-color: #4caf50; color: white; }"
                                "</style>"
                                "</head>"
                                "<body>"
                                "<h1>SMART GARDEN</h1>"
                                "<table>"
                                "<tr>"
                                "<th>Sensor</th>"
                                "<th>Value</th>"
                                "</tr>"
                                "<tr>"
                                "<td>Sensor 1</td>"
                                "<td>%.2f</td>"
                                "</tr>"
                                "<tr>"
                                "<td>Sensor 2</td>"
                                "<td>%.2f</td>"
                                "</tr>"
                                "</table>"
                                "</body>"
                                "</html>";

    // Calculate the size needed for the dynamic HTML content
    size_t content_size = snprintf(NULL, 0, html_template, sensorValue1, sensorValue2) + 1;
    
    // Allocate memory for the dynamic HTML content
    char* dynamic_html_content = (char*)malloc(content_size);

    if (dynamic_html_content == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal Server Error");
        return ESP_FAIL;
    }

    // Format the dynamic HTML content with actual sensor values
    snprintf(dynamic_html_content, content_size, html_template, sensorValue1, sensorValue2);

    // Respond with the dynamic HTML content
    httpd_resp_send(req, dynamic_html_content, HTTPD_RESP_USE_STRLEN);

    // Free the allocated memory
    free(dynamic_html_content);

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = root_get_handler
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char servercert_start[] asm("_binary_servercert_pem_start");
    extern const unsigned char servercert_end[]   asm("_binary_servercert_pem_end");
    conf.servercert = servercert_start;
    conf.servercert_len = servercert_end - servercert_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[]   asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret) {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root);
    return server;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_ssl_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop https server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        *server = start_webserver();
    }
}

void app_main(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start server when Wi-Fi is connected,
     * and stop server when disconnection happens.
     */

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(example_connect());
}
