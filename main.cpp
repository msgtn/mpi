#include <iostream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <algorithm>
#include <vector>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sys/mman.h>

#include <libcamera/libcamera.h>
#include <gpiod.h>
#include <turbojpeg.h>

namespace fs = std::filesystem;
using namespace libcamera;
using namespace std::chrono;

// --- Configuration ---
constexpr int BUTTON_PIN = 23;
constexpr int SCREEN_PIN = 24;
constexpr int LED_PIN = 12;

// Exposure control pins
constexpr int EXPOSURE_PIN_1000 = 19;  // 1/1000 sec
constexpr int EXPOSURE_PIN_250 = 5;    // 1/250 sec
constexpr int EXPOSURE_PIN_60 = 6;     // 1/60 sec
constexpr int EXPOSURE_PIN_15 = 26;    // 1/15 sec

// Gain control pin
constexpr int GAIN_PIN = 20;
// constexpr int WIDTH = 2312;
// constexpr int HEIGHT = 1736;
// constexpr int WIDTH = 3600;
// constexpr int HEIGHT = 2400;
constexpr int WIDTH = 4624;
constexpr int HEIGHT = 3472;
constexpr int JPEG_QUALITY = 90;
const std::string TAPES_DIR = std::string(getenv("HOME")) + "/tapes";

// --- Capture job for async encoding ---
struct CaptureJob {
    std::vector<uint8_t> yuvData;
    int width;
    int height;
    int yStride;
    int uvStride;
    std::string path;
    size_t numPlanes;
    size_t plane0Offset;
    size_t plane1Offset;
    size_t plane2Offset;
};

// --- Global state ---
static std::shared_ptr<Camera> camera;
static std::unique_ptr<CameraManager> cameraManager;
static std::unique_ptr<CameraConfiguration> config;
static FrameBufferAllocator *allocator = nullptr;
static std::vector<std::unique_ptr<Request>> requests;
static std::atomic<bool> running{true};
static std::atomic<time_point<steady_clock>> lastPressed{steady_clock::now() - seconds(2)};
static std::atomic<int> captureCountdown{0};  // Frames to skip before capturing
static std::atomic<int32_t> currentExposureTime{static_cast<int32_t>(1e6 / 30)};  // Default 1/30 sec
static std::atomic<int> currentGainIndex{1};  // Index into gains array (0=2.0, 1=4.0, 2=8.0)
static constexpr float GAIN_VALUES[] = {2.0f, 4.0f, 8.0f};
static std::atomic<time_point<steady_clock>> lastFrameTime{steady_clock::now()};  // Watchdog timer

// --- Encoder thread state ---
static std::thread encoderThread;
static std::mutex captureMutex;
static std::condition_variable captureCV;
static std::queue<CaptureJob> captureQueue;
static tjhandle tjInstance = nullptr;

// --- Helper functions ---
std::string getTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

void runCommand(const std::string &cmd) {
    std::system(cmd.c_str());
}

void setLedPin(bool high) {
    runCommand("raspi-gpio set " + std::to_string(LED_PIN) + (high ? " dh" : " dl"));
}


void turnOffScreen() {
    runCommand("raspi-gpio set 24 op");
    runCommand("raspi-gpio set 24 dl");
    runCommand("raspi-gpio set " + std::to_string(LED_PIN) + " op");
    runCommand("raspi-gpio set " + std::to_string(GAIN_PIN) + " ip");
    runCommand("raspi-gpio set " + std::to_string(EXPOSURE_PIN_1000) + " ip");
    runCommand("raspi-gpio set " + std::to_string(EXPOSURE_PIN_250) + " ip");
    runCommand("raspi-gpio set " + std::to_string(EXPOSURE_PIN_60) + " ip");
    runCommand("raspi-gpio set " + std::to_string(EXPOSURE_PIN_15) + " ip");
    setLedPin(false);
}

void triggerLed() {
    runCommand("raspi-gpio set 47 dh");
}

void setExposureTime(int buttonPin) {
    int32_t exposureTime;
    int nBlinks;
    const char* speedName;

    switch (buttonPin) {
        case EXPOSURE_PIN_1000:
            exposureTime = static_cast<int32_t>(1e6 / 1000);
            speedName = "1/1000";
            nBlinks = 4;
            break;
        case EXPOSURE_PIN_250:
            exposureTime = static_cast<int32_t>(1e6 / 250);
            speedName = "1/250";
            nBlinks = 3;
            break;
        case EXPOSURE_PIN_60:
            exposureTime = static_cast<int32_t>(1e6 / 60);
            speedName = "1/60";
            nBlinks = 2;
            break;
        case EXPOSURE_PIN_15:
            exposureTime = static_cast<int32_t>(1e6 / 15);
            speedName = "1/15";
            nBlinks = 1;
            break;
        default:
            return;

    }

    for (int x = 0; x < nBlinks; x++) {
        setLedPin(true);
        std::this_thread::sleep_for(milliseconds(30));
        setLedPin(false);
        std::this_thread::sleep_for(milliseconds(300));
    }
    currentExposureTime.store(exposureTime);
    std::cout << "Exposure set to " << speedName << " sec (" << exposureTime << " us)" << std::endl;
}

void cycleAnalogueGain() {
    // Cycle to next gain value
    int newIndex = (currentGainIndex.load() + 1) % 3;
    currentGainIndex.store(newIndex);

    float gain = GAIN_VALUES[newIndex];
    int nBlinks = newIndex + 1;  // 1 blink for 2.0, 2 for 4.0, 3 for 8.0

    for (int x = 0; x < nBlinks; x++) {
        setLedPin(true);
        std::this_thread::sleep_for(milliseconds(30));
        setLedPin(false);
        std::this_thread::sleep_for(milliseconds(300));
    }

    std::cout << "Gain set to " << gain << std::endl;
}

// --- Encoder thread function ---
void encoderThreadFunc() {
    while (true) {
        CaptureJob job;
        {
            std::unique_lock<std::mutex> lock(captureMutex);
            captureCV.wait(lock, [] { return !captureQueue.empty() || !running; });

            if (!running && captureQueue.empty()) {
                break;
            }

            job = std::move(captureQueue.front());
            captureQueue.pop();
        }

        // Encode using turbojpeg
        const uint8_t *yPlane = job.yuvData.data() + job.plane0Offset;
        const uint8_t *uPlane = job.yuvData.data() + job.plane1Offset;
        const uint8_t *vPlane = job.yuvData.data() + job.plane2Offset;

        // Build planar YUV buffer for turbojpeg (I420 format)
        // turbojpeg expects contiguous planes without padding
        int ySize = job.width * job.height;
        int uvSize = (job.width / 2) * (job.height / 2);
        std::vector<uint8_t> yuvPlanar(ySize + uvSize * 2);

        // Copy Y plane (removing stride padding)
        for (int y = 0; y < job.height; y++) {
            std::memcpy(&yuvPlanar[y * job.width], &yPlane[y * job.yStride], job.width);
        }

        // Copy U plane
        uint8_t *uDst = yuvPlanar.data() + ySize;
        for (int y = 0; y < job.height / 2; y++) {
            std::memcpy(&uDst[y * (job.width / 2)], &uPlane[y * job.uvStride], job.width / 2);
        }

        // Copy V plane
        uint8_t *vDst = uDst + uvSize;
        for (int y = 0; y < job.height / 2; y++) {
            std::memcpy(&vDst[y * (job.width / 2)], &vPlane[y * job.uvStride], job.width / 2);
        }

        // Compress to JPEG
        unsigned char *jpegBuf = nullptr;
        unsigned long jpegSize = 0;

        int result = tjCompressFromYUV(
            tjInstance,
            yuvPlanar.data(),
            job.width,
            1,  // pad (1 = no padding since we removed it)
            job.height,
            TJSAMP_420,
            &jpegBuf,
            &jpegSize,
            JPEG_QUALITY,
            TJFLAG_FASTDCT
        );

        if (result == 0 && jpegBuf) {
            // Write to file
            FILE *outfile = fopen(job.path.c_str(), "wb");
            if (outfile) {
                fwrite(jpegBuf, 1, jpegSize, outfile);
                fclose(outfile);
                std::cout << "Saved: " << job.path << " (" << jpegSize / 1024 << " KB)" << std::endl;
                setLedPin(true);
                std::this_thread::sleep_for(milliseconds(30));
                setLedPin(false);
            } else {
                std::cerr << "Failed to open output file: " << job.path << std::endl;
            }
            tjFree(jpegBuf);
        } else {
            std::cerr << "JPEG encoding failed: " << tjGetErrorStr2(tjInstance) << std::endl;
        }
    }
}

// --- Request completed callback ---
static void requestComplete(Request *request) {
    if (request->status() == Request::RequestCancelled) {
        return;
    }

    // Check for camera errors - exit so systemd can restart
    if (request->status() != Request::RequestComplete) {
        std::cerr << "Camera error detected, exiting..." << std::endl;
        running = false;
        captureCV.notify_all();
        return;
    }

    // Update watchdog timer
    lastFrameTime.store(steady_clock::now());

    // Countdown mechanism: skip frames to get a fresh, fully-exposed one
    int countdown = captureCountdown.load();
    if (countdown > 0) {
        if (captureCountdown.fetch_sub(1) > 1) {
            // Still counting down, skip this frame
            request->reuse(Request::ReuseBuffers);
            camera->queueRequest(request);
            return;
        }
        // countdown reached 1, capture this frame (falls through)
        const auto &buffers = request->buffers();
        for (auto &bufferPair : buffers) {
            const Stream *stream = bufferPair.first;
            FrameBuffer *buffer = bufferPair.second;
            const auto &planes = buffer->planes();

            if (planes.empty()) {
                std::cerr << "No planes in buffer" << std::endl;
                continue;
            }

            // Get stream configuration
            const StreamConfiguration &streamConfig = stream->configuration();
            int width = streamConfig.size.width;
            int height = streamConfig.size.height;
            int yStride = streamConfig.stride;
            std::string formatStr = streamConfig.pixelFormat.toString();

            // Only support YUV420 for now (most common on Pi)
            if (formatStr != "YUV420") {
                std::cerr << "Unsupported format for fast encoding: " << formatStr << std::endl;
                continue;
            }

            std::cout << "Capture: " << width << "x" << height << " (queuing for encoding)" << std::endl;

            // Calculate total buffer size
            size_t totalSize = 0;
            for (const auto &plane : planes) {
                size_t planeEnd = plane.offset + plane.length;
                if (planeEnd > totalSize) totalSize = planeEnd;
            }

            // Map buffer
            void *mappedBuffer = mmap(nullptr, totalSize, PROT_READ, MAP_SHARED,
                                       planes[0].fd.get(), 0);
            if (mappedBuffer == MAP_FAILED) {
                std::cerr << "mmap failed: " << strerror(errno) << std::endl;
                continue;
            }

            // Create capture job and copy data
            CaptureJob job;
            job.width = width;
            job.height = height;
            job.yStride = yStride;
            job.uvStride = yStride / 2;
            job.path = TAPES_DIR + "/mpi_" + getTimestamp() + ".jpg";
            job.numPlanes = planes.size();
            job.yuvData.resize(totalSize);
            std::memcpy(job.yuvData.data(), mappedBuffer, totalSize);

            // Set plane offsets
            job.plane0Offset = planes[0].offset;
            if (planes.size() >= 3) {
                job.plane1Offset = planes[1].offset;
                job.plane2Offset = planes[2].offset;
            } else if (planes.size() == 1) {
                // Single plane - calculate offsets
                job.plane1Offset = yStride * height;
                job.plane2Offset = job.plane1Offset + (yStride / 2) * (height / 2);
            }

            munmap(mappedBuffer, totalSize);

            // Queue job for encoding thread
            {
                std::lock_guard<std::mutex> lock(captureMutex);
                captureQueue.push(std::move(job));
            }
            captureCV.notify_one();

            triggerLed();
        }
    }

    // Re-queue the request with current exposure and gain
    request->reuse(Request::ReuseBuffers);
    request->controls().set(controls::ExposureTime, currentExposureTime.load());
    request->controls().set(controls::AnalogueGain, GAIN_VALUES[currentGainIndex.load()]);
    if (camera->queueRequest(request) < 0) {
        std::cerr << "Failed to re-queue request, exiting..." << std::endl;
        running = false;
        captureCV.notify_all();
    }
}

// --- Camera cleanup ---
void cleanupCamera() {
    if (camera) {
        camera->stop();
        camera->requestCompleted.disconnect(requestComplete);
        camera->release();
        camera.reset();
    }
    if (allocator) {
        delete allocator;
        allocator = nullptr;
    }
    requests.clear();
    if (cameraManager) {
        cameraManager->stop();
        cameraManager.reset();
    }
    config.reset();
}

// --- Camera setup ---
bool setupCamera() {
    cameraManager = std::make_unique<CameraManager>();
    if (cameraManager->start()) {
        std::cerr << "Failed to start camera manager" << std::endl;
        return false;
    }

    if (cameraManager->cameras().empty()) {
        std::cerr << "No cameras found" << std::endl;
        return false;
    }

    camera = cameraManager->cameras()[0];
    if (camera->acquire()) {
        std::cerr << "Failed to acquire camera" << std::endl;
        return false;
    }

    // Configure camera
    config = camera->generateConfiguration({StreamRole::StillCapture});
    if (!config) {
        std::cerr << "Failed to generate configuration" << std::endl;
        return false;
    }

    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size.width = WIDTH;
    streamConfig.size.height = HEIGHT;
    streamConfig.bufferCount = 1;

    if (config->validate() == CameraConfiguration::Invalid) {
        std::cerr << "Invalid camera configuration" << std::endl;
        return false;
    }

    if (camera->configure(config.get())) {
        std::cerr << "Failed to configure camera" << std::endl;
        return false;
    }

    // Allocate buffers
    allocator = new FrameBufferAllocator(camera);
    Stream *stream = streamConfig.stream();

    if (allocator->allocate(stream) < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        return false;
    }

    // Create requests
    for (const std::unique_ptr<FrameBuffer> &buffer : allocator->buffers(stream)) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            return false;
        }
        if (request->addBuffer(stream, buffer.get())) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            return false;
        }
        requests.push_back(std::move(request));
    }

    // Connect signal and start camera
    camera->requestCompleted.connect(requestComplete);

    // Set up initial controls
    ControlList startControls;
    startControls.set(controls::AeEnable, false);
    startControls.set(controls::ExposureTime, static_cast<int32_t>(1e6 / 30));
    startControls.set(controls::AnalogueGain, 4.0f);

    if (camera->start(&startControls)) {
        std::cerr << "Failed to start camera" << std::endl;
        return false;
    }

    // Queue all requests with controls
    for (auto &request : requests) {
        request->controls().set(controls::AeEnable, false);
        request->controls().set(controls::ExposureTime, static_cast<int32_t>(1e6 / 30));
        request->controls().set(controls::AnalogueGain, 4.0f);
        camera->queueRequest(request.get());
    }

    std::cout << "Camera initialized: " << WIDTH << "x" << HEIGHT << std::endl;
    return true;
}

// --- GPIO button handling ---
void buttonThread() {
    // Try different chip names (gpiochip4 for Pi 5, gpiochip0 for Pi 4 and earlier)
    const char *chipNames[] = {"gpiochip4", "gpiochip0", "pinctrl-bcm2835", nullptr};
    struct gpiod_chip *chip = nullptr;

    for (int i = 0; chipNames[i] != nullptr; i++) {
        chip = gpiod_chip_open_by_name(chipNames[i]);
        if (chip) {
            std::cout << "Opened GPIO chip: " << chipNames[i] << std::endl;
            break;
        }
    }

    if (!chip) {
        std::cerr << "Failed to open any GPIO chip" << std::endl;
        return;
    }

    // All pins to monitor
    const int pins[] = {BUTTON_PIN, EXPOSURE_PIN_1000, EXPOSURE_PIN_250, EXPOSURE_PIN_60, EXPOSURE_PIN_15, GAIN_PIN};
    const int numPins = sizeof(pins) / sizeof(pins[0]);
    struct gpiod_line *lines[numPins];
    struct gpiod_line_bulk bulk;

    gpiod_line_bulk_init(&bulk);

    for (int i = 0; i < numPins; i++) {
        lines[i] = gpiod_chip_get_line(chip, pins[i]);
        if (!lines[i]) {
            std::cerr << "Failed to get GPIO line " << pins[i] << std::endl;
            gpiod_chip_close(chip);
            return;
        }
        gpiod_line_bulk_add(&bulk, lines[i]);
    }

    // Request with pull-up bias and falling edge events
    struct gpiod_line_request_config config = {
        .consumer = "picam-button",
        .request_type = GPIOD_LINE_REQUEST_EVENT_FALLING_EDGE,
        .flags = GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP,
    };

    if (gpiod_line_request_bulk(&bulk, &config, nullptr) < 0) {
        std::cerr << "Failed to request GPIO lines (error: " << strerror(errno) << ")" << std::endl;
        gpiod_chip_close(chip);
        return;
    }

    std::cout << "Button monitoring started on GPIOs: " << BUTTON_PIN
              << ", " << EXPOSURE_PIN_1000 << ", " << EXPOSURE_PIN_250
              << ", " << EXPOSURE_PIN_60 << ", " << EXPOSURE_PIN_15
              << ", " << GAIN_PIN << std::endl;

    while (running) {
        struct gpiod_line_bulk eventBulk;
        struct timespec timeout = {0, 100000000}; // 100ms timeout

        int ret = gpiod_line_event_wait_bulk(&bulk, &timeout, &eventBulk);
        if (ret < 0) {
            std::cerr << "Error waiting for GPIO event" << std::endl;
            break;
        }
        if (ret > 0) {
            // Process events from all lines that triggered
            for (unsigned int i = 0; i < gpiod_line_bulk_num_lines(&eventBulk); i++) {
                struct gpiod_line *line = gpiod_line_bulk_get_line(&eventBulk, i);
                struct gpiod_line_event event;
                gpiod_line_event_read(line, &event);

                int pin = gpiod_line_offset(line);
                auto now = steady_clock::now();
                auto last = lastPressed.load();

                // Debounce: ignore presses within 300ms
                if (duration_cast<milliseconds>(now - last).count() > 300) {
                    lastPressed.store(now);

                    if (pin == BUTTON_PIN) {
                        // Check if capture already in progress
                        bool busy = false;
                        {
                            std::lock_guard<std::mutex> lock(captureMutex);
                            busy = !captureQueue.empty() || captureCountdown.load() > 0;
                        }
                        if (busy) {
                            std::cout << "Capture busy, ignoring button press" << std::endl;
                        } else {
                            std::cout << "Button pressed, capturing..." << std::endl;
                            captureCountdown.store(3);
                        }
                    // } else if (pin == GAIN_PIN) {
                    //     cycleAnalogueGain();
                    } else {
                        setExposureTime(pin);
                    }
                }
            }
        }
    }

    gpiod_line_release_bulk(&bulk);
    gpiod_chip_close(chip);
}

// --- Signal handler ---
void signalHandler(int sig) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
    captureCV.notify_all();  // Wake up encoder thread
}

// --- Main ---
int main() {
    // Setup signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Initialize turbojpeg
    tjInstance = tjInitCompress();
    if (!tjInstance) {
        std::cerr << "Failed to initialize turbojpeg" << std::endl;
        return 1;
    }

    // Turn off screen
    turnOffScreen();

    // Create tapes directory
    fs::create_directories(TAPES_DIR);

    // Start encoder thread
    encoderThread = std::thread(encoderThreadFunc);

    // Setup camera
    if (!setupCamera()) {
        running = false;
        captureCV.notify_all();
        encoderThread.join();
        tjDestroy(tjInstance);
        return 1;
    }

    // Start button monitoring thread
    std::thread buttonMonitor(buttonThread);

    std::cout << "Ready. Waiting for button press..." << std::endl;

    // Main loop with watchdog
    while (running) {
        // Check if camera has timed out (no frames for 5 seconds)
        auto timeSinceLastFrame = duration_cast<seconds>(steady_clock::now() - lastFrameTime.load()).count();
        if (timeSinceLastFrame > 5) {
            std::cerr << "Camera watchdog timeout - no frames for " << timeSinceLastFrame << " seconds, exiting..." << std::endl;
            break;
        }
        std::this_thread::sleep_for(milliseconds(100));
    }

    // Cleanup
    buttonMonitor.join();

    // Wait for encoder to finish pending jobs
    captureCV.notify_all();
    encoderThread.join();

    cleanupCamera();
    tjDestroy(tjInstance);

    std::cout << "Goodbye!" << std::endl;
    return 0;
}
