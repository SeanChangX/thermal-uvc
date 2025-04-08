// Include necessary libraries
#include <libuvc/libuvc.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <chrono>

// Global constants
const char* WINDOW_NAME = "Thermal_Camera";
double scale_factor = 2.0; // Default scaling factor

// Callback function for the trackbar
void onTrackbar(int pos, void*) {
    scale_factor = pos / 10.0;
    if (scale_factor < 0.1) scale_factor = 0.1; // Minimum scaling
}

// Callback function to process each frame from the UVC device
void uvc_frame_callback(uvc_frame_t *frame, void *ptr) {
    // Allocate memory for BGR frame
    uvc_frame_t *bgr = uvc_allocate_frame(frame->width * frame->height * 3);
    if (!bgr) return;

    // Convert frame to BGR format
    uvc_error_t err = uvc_any2bgr(frame, bgr);
    if (err != UVC_SUCCESS) {
        uvc_free_frame(bgr);
        return;
    }

    // Create OpenCV Mat from the BGR frame
    cv::Mat img(cv::Size(bgr->width, bgr->height), CV_8UC3, bgr->data);

    // Apply a color map to enhance visualization
    cv::Mat colored;
    cv::applyColorMap(img, colored, cv::COLORMAP_JET);
    
    // Scale the image while maintaining aspect ratio
    cv::Mat resized;
    cv::resize(colored, resized, cv::Size(0, 0), scale_factor, scale_factor, cv::INTER_LINEAR);
    
    // Display the scaled thermal image
    cv::imshow(WINDOW_NAME, resized);

    // Exit if ESC key is pressed
    if (cv::waitKey(1) == 27) {
        std::cout << "ESC pressed. Exiting..." << std::endl;
        exit(0);
    }

    // Free the allocated frame
    uvc_free_frame(bgr);
}

// Function to print available formats for a given device
void print_device_formats(uvc_device_handle_t *devh) {
    std::cout << "Available formats:" << std::endl;
    
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
    while (format_desc) {
        std::cout << "Format: " << format_desc->bFormatIndex 
                  << ", Type: " << format_desc->bDescriptorSubtype << std::endl;
        
        const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
        while (frame_desc) {
            std::cout << "  " << frame_desc->wWidth << "x" << frame_desc->wHeight 
                      << " @ " << (10000000 / frame_desc->dwDefaultFrameInterval) << " fps" 
                      << std::endl;
            frame_desc = frame_desc->next;
        }
        format_desc = format_desc->next;
    }
}

// Function to attempt streaming with different formats
bool try_streaming_formats(uvc_device_handle_t *devh, uvc_stream_ctrl_t *ctrl) {
    // First try with specific resolution
    if (uvc_get_stream_ctrl_format_size(devh, ctrl, UVC_FRAME_FORMAT_YUYV, 256, 192, 25) == 0) {
        std::cout << "Using format: YUYV 256x192 @ 25fps" << std::endl;
        return true;
    }
    
    // Try with other common formats and resolutions
    const uvc_format_desc_t *format_desc = uvc_get_format_descs(devh);
    while (format_desc) {
        const uvc_frame_desc_t *frame_desc = format_desc->frame_descs;
        while (frame_desc) {
            // Try with the device's native format and resolution
            if (uvc_get_stream_ctrl_format_size(devh, ctrl, 
                    UVC_FRAME_FORMAT_ANY, // Try with any format
                    frame_desc->wWidth, 
                    frame_desc->wHeight, 
                    0) == 0) { // 0 = any frame rate
                
                std::cout << "Using device's native format: " 
                          << frame_desc->wWidth << "x" << frame_desc->wHeight << std::endl;
                return true;
            }
            frame_desc = frame_desc->next;
        }
        format_desc = format_desc->next;
    }
    
    return false;
}

int main() {
    // Initialize UVC context
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;

    if (uvc_init(&ctx, NULL) < 0) {
        std::cerr << "Failed to init libuvc" << std::endl;
        return -1;
    }

    // Find the UVC device (replace VID/PID with your device's values)
    if (uvc_find_device(ctx, &dev, 0x2bdf, 0x0102, NULL) < 0) {
        std::cerr << "Device not found" << std::endl;
        uvc_exit(ctx);
        return -1;
    }

    // Open the UVC device
    if (uvc_open(dev, &devh) < 0) {
        std::cerr << "Failed to open device" << std::endl;
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    std::cout << "Device opened!" << std::endl;

    // Print available formats
    print_device_formats(devh);

    // Try to find a compatible format
    if (!try_streaming_formats(devh, &ctrl)) {
        std::cerr << "Couldn't find a compatible streaming format!" << std::endl;
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    // Start streaming
    if (uvc_start_streaming(devh, &ctrl, uvc_frame_callback, nullptr, 0) < 0) {
        std::cerr << "Failed to start streaming!" << std::endl;
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return -1;
    }

    std::cout << "Streaming started. Press ESC to quit." << std::endl;

    // Create a window and a trackbar for scaling
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
    cv::createTrackbar("Scale x10", WINDOW_NAME, nullptr, 50, onTrackbar);
    cv::setTrackbarPos("Scale x10", WINDOW_NAME, static_cast<int>(scale_factor * 10));

    // Keep the program running to process frames
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Stop streaming and clean up
    uvc_stop_streaming(devh);
    uvc_close(devh);
    uvc_unref_device(dev);
    uvc_exit(ctx);
    return 0;
}
