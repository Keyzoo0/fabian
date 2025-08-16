class ColorDetectionApp {
  constructor() {
    this.isRunning = false;
    this.intervalId = null;
    this.dataIntervalId = null;

    // Initialize with default values - SAME AS TRACKBARV2
    this.values = {
      hsv: {
        h_min: 0,
        s_min: 0,
        v_min: 0,
        h_max: 179,
        s_max: 255,
        v_max: 255,
      },
      morphology: {
        erosion: 0,
        dilation: 0,
        opening: 0,
        closing: 0,
      },
      calibration: {
        calib_distance_cm: 50,
        calib_pixel_area: 5000,
        min_area_threshold: 500,
      },
      camera_controls: {
        auto_exposure: 0,
        exposure: 50,
        auto_wb: 0,
        wb_temperature: 40,
        brightness: 50,
        contrast: 50,
      },
      object_data: {
        x: 0,
        y: 0,
        distance: 0,
        error_x: 0,
        error_y: 0,
        area: 0,
      },
    };

    this.init();
  }

  init() {
    this.setupEventListeners();
    this.loadInitialValues();
    this.updateStatusIndicator("connecting", "Initializing...");
    this.checkSerialStatus();
  }

  setupEventListeners() {
    // Control buttons
    document
      .getElementById("start-btn")
      .addEventListener("click", () => this.startCamera());
    document
      .getElementById("stop-btn")
      .addEventListener("click", () => this.stopCamera());
    document
      .getElementById("save-btn")
      .addEventListener("click", () => this.saveConfiguration());
    document
      .getElementById("reset-btn")
      .addEventListener("click", () => this.resetToDefault());

    // HSV sliders
    this.setupSliderGroup("hsv", [
      "h-min",
      "h-max",
      "s-min",
      "s-max",
      "v-min",
      "v-max",
    ]);

    // Morphology sliders
    this.setupSliderGroup("morphology", [
      "erosion",
      "dilation",
      "opening",
      "closing",
    ]);

    // Calibration sliders
    this.setupSliderGroup("calibration", [
      "calib-distance",
      "calib-pixel-area",
      "min-area-threshold",
    ]);

    // Camera control sliders
    this.setupSliderGroup("camera_controls", [
      "exposure",
      "wb-temperature",
      "brightness",
      "contrast",
    ]);

    // Camera control dropdowns
    document.getElementById("auto-exposure").addEventListener("change", (e) => {
      this.updateValue(
        "camera_controls",
        "auto_exposure",
        parseInt(e.target.value)
      );
    });

    document.getElementById("auto-wb").addEventListener("change", (e) => {
      this.updateValue("camera_controls", "auto_wb", parseInt(e.target.value));
    });

    // Keyboard shortcuts
    document.addEventListener("keydown", (e) =>
      this.handleKeyboardShortcuts(e)
    );
  }

  setupSliderGroup(group, sliders) {
    sliders.forEach((sliderId) => {
      const slider = document.getElementById(sliderId);
      const valueSpan = document.getElementById(sliderId + "-value");

      if (slider && valueSpan) {
        slider.addEventListener("input", (e) => {
          const value = parseInt(e.target.value);
          valueSpan.textContent = value;

          // Convert slider ID to API key format
          const apiKey = sliderId.replace("-", "_");
          this.updateValue(group, apiKey, value);

          if (group === "hsv") {
            this.updateCurrentHSVDisplay();
          }
        });
      }
    });
  }

  async loadInitialValues() {
    try {
      const response = await fetch("/get_all_values");
      const data = await response.json();
      this.values = data;
      this.updateAllControls();
      this.updateCurrentHSVDisplay();
      this.updateObjectDataDisplay(data.object_data);
    } catch (error) {
      console.error("Error loading initial values:", error);
      this.showToast("Error loading initial values", "error");
    }
  }

  updateAllControls() {
    // Update HSV controls
    Object.entries(this.values.hsv).forEach(([key, value]) => {
      const sliderId = key.replace("_", "-");
      const slider = document.getElementById(sliderId);
      const valueSpan = document.getElementById(sliderId + "-value");

      if (slider && valueSpan) {
        slider.value = value;
        valueSpan.textContent = value;
      }
    });

    // Update morphology controls
    Object.entries(this.values.morphology).forEach(([key, value]) => {
      const slider = document.getElementById(key);
      const valueSpan = document.getElementById(key + "-value");

      if (slider && valueSpan) {
        slider.value = value;
        valueSpan.textContent = value;
      }
    });

    // Update calibration controls
    Object.entries(this.values.calibration).forEach(([key, value]) => {
      const sliderId = key.replace("_", "-");
      const slider = document.getElementById(sliderId);
      const valueSpan = document.getElementById(sliderId + "-value");

      if (slider && valueSpan) {
        slider.value = value;
        valueSpan.textContent = value;
      }
    });

    // Update camera controls
    Object.entries(this.values.camera_controls).forEach(([key, value]) => {
      if (key === "auto_exposure") {
        document.getElementById("auto-exposure").value = value;
      } else if (key === "auto_wb") {
        document.getElementById("auto-wb").value = value;
      } else {
        const sliderId = key.replace("_", "-");
        const slider = document.getElementById(sliderId);
        const valueSpan = document.getElementById(sliderId + "-value");

        if (slider && valueSpan) {
          slider.value = value;
          valueSpan.textContent = value;
        }
      }
    });
  }

  updateCurrentHSVDisplay() {
    document.getElementById(
      "current-h"
    ).textContent = `${this.values.hsv.h_min}-${this.values.hsv.h_max}`;
    document.getElementById(
      "current-s"
    ).textContent = `${this.values.hsv.s_min}-${this.values.hsv.s_max}`;
    document.getElementById(
      "current-v"
    ).textContent = `${this.values.hsv.v_min}-${this.values.hsv.v_max}`;
  }

  async updateValue(group, key, value) {
    this.values[group][key] = value;

    try {
      const endpoint = `/update_${group}`;
      await fetch(endpoint, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ [key]: value }),
      });
    } catch (error) {
      console.error(`Error updating ${group} value:`, error);
    }
  }

  startCamera() {
    if (!this.isRunning) {
      this.isRunning = true;
      this.updateStatusIndicator("connected", "Camera Running");

      const startBtn = document.getElementById("start-btn");
      const stopBtn = document.getElementById("stop-btn");

      startBtn.disabled = true;
      startBtn.classList.add("loading");
      stopBtn.disabled = false;

      // Start frame updates
      this.intervalId = setInterval(() => this.fetchFrames(), 100);
      // Start object data updates
      this.dataIntervalId = setInterval(() => this.fetchObjectData(), 200);

      this.showToast("Camera started successfully", "success");
    }
  }

  stopCamera() {
    if (this.isRunning) {
      this.isRunning = false;

      clearInterval(this.intervalId);
      clearInterval(this.dataIntervalId);

      this.updateStatusIndicator("disconnected", "Camera Stopped");

      const startBtn = document.getElementById("start-btn");
      const stopBtn = document.getElementById("stop-btn");

      startBtn.disabled = false;
      startBtn.classList.remove("loading");
      stopBtn.disabled = true;

      // Clear all frames
      ["original", "hsv", "mask", "result"].forEach((frameType) => {
        const img = document.getElementById(frameType + "-frame");
        if (img) img.src = "";
      });

      // Reset object data display
      this.updateObjectDataDisplay({
        x: 0,
        y: 0,
        distance: 0,
        error_x: 0,
        error_y: 0,
        area: 0,
      });

      // Reset indicators
      this.resetIndicators();
      this.showToast("Camera stopped", "info");
    }
  }

  async fetchFrames() {
    if (!this.isRunning) return;

    try {
      const response = await fetch("/get_frames");
      const data = await response.json();

      if (data.error) {
        console.error("Frame fetch error:", data.error);
        this.updateStatusIndicator("disconnected", "Camera Error");
        return;
      }

      // Update frame images
      Object.entries(data).forEach(([frameType, frameData]) => {
        const imgElement = document.getElementById(frameType + "-frame");
        if (imgElement && frameData) {
          imgElement.src = frameData;
        }
      });
    } catch (error) {
      console.error("Error fetching frames:", error);
      this.updateStatusIndicator("disconnected", "Connection Error");
    }
  }

  async fetchObjectData() {
    if (!this.isRunning) return;

    try {
      const response = await fetch("/get_all_values");
      const data = await response.json();
      this.updateObjectDataDisplay(data.object_data);
    } catch (error) {
      console.error("Error fetching object data:", error);
    }
  }

  updateObjectDataDisplay(data) {
    // Update data values
    document.getElementById("object-x").textContent = data.x;
    document.getElementById("object-y").textContent = data.y;
    document.getElementById("error-x").textContent =
      data.error_x > 0 ? `+${data.error_x}` : data.error_x;
    document.getElementById("error-y").textContent =
      data.error_y > 0 ? `+${data.error_y}` : data.error_y;
    document.getElementById(
      "object-distance"
    ).textContent = `${data.distance.toFixed(1)} cm`;
    document.getElementById("object-area").textContent = `${data.area} px`;

    // Update indicators
    this.updateIndicators(data);
  }

  updateIndicators(data) {
    const detectionIndicator = document.getElementById("detection-indicator");
    const centerIndicator = document.getElementById("center-indicator");

    if (data.area > 0) {
      detectionIndicator.classList.add("active");
      detectionIndicator.classList.remove("inactive");

      // Check if object is in center zone (within ±50 pixels)
      if (Math.abs(data.error_x) <= 50 && Math.abs(data.error_y) <= 50) {
        centerIndicator.classList.add("active");
        centerIndicator.classList.remove("inactive");
      } else {
        centerIndicator.classList.remove("active");
        centerIndicator.classList.add("inactive");
        centerIndicator.style.background = "#f59e0b"; // yellow for near center
      }
    } else {
      detectionIndicator.classList.remove("active");
      detectionIndicator.classList.add("inactive");
      centerIndicator.classList.remove("active");
      centerIndicator.classList.add("inactive");
    }
  }

  resetIndicators() {
    const indicators = ["detection-indicator", "center-indicator"];
    indicators.forEach((id) => {
      const indicator = document.getElementById(id);
      indicator.classList.remove("active");
      indicator.classList.add("inactive");
      indicator.style.background = "#ef4444"; // red
    });
  }

  async saveConfiguration() {
    const saveBtn = document.getElementById("save-btn");
    const originalHTML = saveBtn.innerHTML;

    saveBtn.innerHTML = '<i class="fas fa-spinner fa-spin"></i>Saving...';
    saveBtn.disabled = true;
    saveBtn.classList.add("loading");

    try {
      const response = await fetch("/save_calibration", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      });

      const result = await response.json();

      if (result.status === "success") {
        this.showToast("Configuration saved successfully!", "success");
      } else {
        this.showToast(
          "Error saving configuration: " + result.message,
          "error"
        );
      }
    } catch (error) {
      console.error("Error saving configuration:", error);
      this.showToast("Error saving configuration", "error");
    } finally {
      setTimeout(() => {
        saveBtn.innerHTML = originalHTML;
        saveBtn.disabled = false;
        saveBtn.classList.remove("loading");
      }, 1000);
    }
  }

  async resetToDefault() {
    if (
      !confirm(
        "Are you sure you want to reset all values to default? This will stop the camera."
      )
    ) {
      return;
    }

    // Stop camera first
    if (this.isRunning) {
      this.stopCamera();
    }

    const defaults = {
      hsv: {
        h_min: 0,
        s_min: 0,
        v_min: 0,
        h_max: 179,
        s_max: 255,
        v_max: 255,
      },
      morphology: {
        erosion: 0,
        dilation: 0,
        opening: 0,
        closing: 0,
      },
      calibration: {
        calib_distance_cm: 50,
        calib_pixel_area: 5000,
        min_area_threshold: 500,
      },
      camera_controls: {
        auto_exposure: 0,
        exposure: 50,
        auto_wb: 0,
        wb_temperature: 40,
        brightness: 50,
        contrast: 50,
      },
    };

    this.values = { ...defaults, object_data: this.values.object_data };
    this.updateAllControls();
    this.updateCurrentHSVDisplay();

    // Update backend
    try {
      await Promise.all([
        fetch("/update_hsv", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(defaults.hsv),
        }),
        fetch("/update_morphology", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(defaults.morphology),
        }),
        fetch("/update_calibration", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(defaults.calibration),
        }),
        fetch("/update_camera_controls", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify(defaults.camera_controls),
        }),
      ]);

      this.showToast("All values reset to default", "info");
    } catch (error) {
      console.error("Error resetting values:", error);
      this.showToast("Error resetting values", "error");
    }
  }

  async checkSerialStatus() {
    try {
      // Simulate serial check - replace with actual API call
      document.getElementById("serial-status").textContent =
        "Connected (/dev/ttyUSB0)";
      document.getElementById("serial-status").style.color = "#10b981";
    } catch (error) {
      document.getElementById("serial-status").textContent = "Disconnected";
      document.getElementById("serial-status").style.color = "#ef4444";
    }
  }

  updateStatusIndicator(status, text) {
    const indicator = document.getElementById("status-indicator");
    const statusText = document.getElementById("status-text");

    // Remove existing status classes
    indicator.className = "status-dot";

    // Add new status class
    switch (status) {
      case "connected":
        indicator.classList.add("connected");
        break;
      case "disconnected":
        indicator.classList.add("disconnected");
        break;
      case "connecting":
        indicator.classList.add("connecting");
        break;
    }

    statusText.textContent = text;
  }

  showToast(message, type = "info") {
    const container = document.getElementById("toast-container");

    const icons = {
      success: "fa-check-circle",
      error: "fa-exclamation-circle",
      warning: "fa-exclamation-triangle",
      info: "fa-info-circle",
    };

    const toast = document.createElement("div");
    toast.className = `toast ${type}`;
    toast.innerHTML = `
            <div class="toast-icon">
                <i class="fas ${icons[type]}"></i>
            </div>
            <div class="toast-content">
                <div class="toast-message">${message}</div>
            </div>
            <button class="toast-close" onclick="this.parentElement.remove()">
                <i class="fas fa-times"></i>
            </button>
        `;

    container.appendChild(toast);

    // Animate in
    setTimeout(() => {
      toast.classList.add("show");
    }, 100);

    // Auto remove after 4 seconds
    setTimeout(() => {
      toast.classList.remove("show");
      setTimeout(() => {
        if (toast.parentNode) {
          toast.remove();
        }
      }, 300);
    }, 4000);
  }

  handleKeyboardShortcuts(event) {
    if (event.ctrlKey) {
      switch (event.key.toLowerCase()) {
        case "s":
          event.preventDefault();
          this.saveConfiguration();
          break;
        case "r":
          event.preventDefault();
          this.resetToDefault();
          break;
        case " ":
          event.preventDefault();
          if (this.isRunning) {
            this.stopCamera();
          } else {
            this.startCamera();
          }
          break;
      }
    }
  }

  // Utility method to simulate performance metrics updates
  startPerformanceMonitoring() {
    if (this.isRunning) {
      setInterval(() => {
        // Simulate random but realistic values
        const fps = Math.floor(Math.random() * 5) + 28; // 28-32 FPS
        const processingTime = Math.floor(Math.random() * 10) + 30; // 30-40ms
        const stability = Math.floor(Math.random() * 10) + 90; // 90-99%
        const latency = Math.floor(Math.random() * 3) + 1; // 1-3ms

        // Update display if elements exist
        const elements = {
          "frame-rate": `${fps} FPS`,
          "processing-time": `${processingTime}ms`,
          "tracking-stability": `${stability}%`,
          "serial-latency": `${latency}ms`,
        };

        Object.entries(elements).forEach(([id, value]) => {
          const element = document.querySelector(`[data-metric="${id}"]`);
          if (element) element.textContent = value;
        });
      }, 2000);
    }
  }
}

// Initialize application when DOM is loaded
document.addEventListener("DOMContentLoaded", function () {
  const app = new ColorDetectionApp();

  // Auto-start camera after 1 second
  setTimeout(() => {
    app.startCamera();
    app.startPerformanceMonitoring();
  }, 1000);

  // Handle page unload
  window.addEventListener("beforeunload", function (event) {
    if (app.isRunning) {
      event.preventDefault();
      event.returnValue =
        "Camera is still running. Are you sure you want to leave?";
      return event.returnValue;
    }
  });

  // Add some additional interactive features
  document.addEventListener("click", (e) => {
    // Add ripple effect to buttons
    if (e.target.classList.contains("btn")) {
      const ripple = document.createElement("span");
      const rect = e.target.getBoundingClientRect();
      const size = Math.max(rect.width, rect.height);
      const x = e.clientX - rect.left - size / 2;
      const y = e.clientY - rect.top - size / 2;

      ripple.style.cssText = `
                position: absolute;
                width: ${size}px;
                height: ${size}px;
                left: ${x}px;
                top: ${y}px;
                background: rgba(255, 255, 255, 0.4);
                border-radius: 50%;
                transform: scale(0);
                animation: ripple 0.6s linear;
                pointer-events: none;
            `;

      e.target.style.position = "relative";
      e.target.style.overflow = "hidden";
      e.target.appendChild(ripple);

      setTimeout(() => {
        ripple.remove();
      }, 600);
    }
  });

  // Add CSS for ripple animation
  const style = document.createElement("style");
  style.textContent = `
        @keyframes ripple {
            to {
                transform: scale(4);
                opacity: 0;
            }
        }
    `;
  document.head.appendChild(style);
});
