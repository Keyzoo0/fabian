class ColorDetectionApp {
  constructor() {
    this.isRunning = false;
    this.intervalId = null;
    this.dataIntervalId = null;

    // Initialize with default values - EXACT SAME AS TRACKBARV2
    this.values = {
      hsv: { h_min: 0, s_min: 0, v_min: 0, h_max: 179, s_max: 255, v_max: 255 },
      morphology: { erosion: 0, dilation: 0, opening: 0, closing: 0 },
      calibration: {
        calib_distance_cm: 50,
        calib_pixel_area: 5000, // EXACT SAME AS TRACKBARV2
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
      object_data: { x: 0, y: 0, distance: 0, error_x: 0, error_y: 0, area: 0 },
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
        const dropdown = document.getElementById("auto-exposure");
        if (dropdown) dropdown.value = value;
      } else if (key === "auto_wb") {
        const dropdown = document.getElementById("auto-wb");
        if (dropdown) dropdown.value = value;
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
    const currentH = document.getElementById("current-h");
    const currentS = document.getElementById("current-s");
    const currentV = document.getElementById("current-v");

    if (currentH)
      currentH.textContent = `${this.values.hsv.h_min}-${this.values.hsv.h_max}`;
    if (currentS)
      currentS.textContent = `${this.values.hsv.s_min}-${this.values.hsv.s_max}`;
    if (currentV)
      currentV.textContent = `${this.values.hsv.v_min}-${this.values.hsv.v_max}`;
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

      document.getElementById("start-btn").disabled = true;
      document.getElementById("stop-btn").disabled = false;

      // Start frame updates (faster for smooth video)
      this.intervalId = setInterval(() => this.fetchFrames(), 100);
      // Start object data updates (slower for efficiency)
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

      document.getElementById("start-btn").disabled = false;
      document.getElementById("stop-btn").disabled = true;

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
      const detectionIndicator = document.getElementById("detection-indicator");
      const centerIndicator = document.getElementById("center-indicator");

      if (detectionIndicator)
        detectionIndicator.className = "w-3 h-3 rounded-full bg-red-400";
      if (centerIndicator)
        centerIndicator.className = "w-3 h-3 rounded-full bg-red-400";

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
    // Update object data display
    const elements = {
      "object-x": data.x,
      "object-y": data.y,
      "error-x": data.error_x > 0 ? `+${data.error_x}` : data.error_x,
      "error-y": data.error_y > 0 ? `+${data.error_y}` : data.error_y,
      "object-distance": `${data.distance.toFixed(1)} cm`,
      "object-area": `${data.area} px`,
    };

    Object.entries(elements).forEach(([id, value]) => {
      const element = document.getElementById(id);
      if (element) element.textContent = value;
    });

    // Update status indicators
    const detectionIndicator = document.getElementById("detection-indicator");
    const centerIndicator = document.getElementById("center-indicator");

    if (detectionIndicator && centerIndicator) {
      if (data.area > 0) {
        detectionIndicator.className = "w-3 h-3 rounded-full bg-green-400";

        // Check if object is in center zone (within ±50 pixels) - SAME AS TRACKBARV2 LOGIC
        if (Math.abs(data.error_x) <= 50 && Math.abs(data.error_y) <= 50) {
          centerIndicator.className = "w-3 h-3 rounded-full bg-green-400";
        } else {
          centerIndicator.className = "w-3 h-3 rounded-full bg-yellow-400";
        }
      } else {
        detectionIndicator.className = "w-3 h-3 rounded-full bg-red-400";
        centerIndicator.className = "w-3 h-3 rounded-full bg-red-400";
      }
    }
  }

  async saveConfiguration() {
    const saveBtn = document.getElementById("save-btn");
    const originalHTML = saveBtn.innerHTML;

    saveBtn.innerHTML = '<i class="fas fa-spinner fa-spin mr-1"></i>Saving...';
    saveBtn.disabled = true;

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

    // DEFAULT VALUES SAME AS TRACKBARV2
    const defaults = {
      hsv: { h_min: 0, s_min: 0, v_min: 0, h_max: 179, s_max: 255, v_max: 255 },
      morphology: { erosion: 0, dilation: 0, opening: 0, closing: 0 },
      calibration: {
        calib_distance_cm: 50,
        calib_pixel_area: 5000, // TRACKBARV2 DEFAULT VALUE
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

      this.showToast(
        "All values reset to default (exact trackbarv2 match)",
        "info"
      );
    } catch (error) {
      console.error("Error resetting values:", error);
      this.showToast("Error resetting values", "error");
    }
  }

  async checkSerialStatus() {
    try {
      // Simulate serial status check - could be enhanced with real API endpoint
      const serialStatus = document.getElementById("serial-status");
      if (serialStatus) {
        serialStatus.textContent = "Connected (/dev/ttyUSB0)";
        serialStatus.className = "ml-1 text-green-400";
      }
    } catch (error) {
      const serialStatus = document.getElementById("serial-status");
      if (serialStatus) {
        serialStatus.textContent = "Disconnected";
        serialStatus.className = "ml-1 text-red-400";
      }
    }
  }

  updateStatusIndicator(status, text) {
    const indicator = document.getElementById("status-indicator");
    const statusText = document.getElementById("status-text");

    if (indicator && statusText) {
      // Remove existing status classes
      indicator.className = indicator.className.replace(/(bg-\w+-\d+)/g, "");

      // Add new status class
      switch (status) {
        case "connected":
          indicator.classList.add("bg-green-400");
          break;
        case "disconnected":
          indicator.classList.add("bg-red-400");
          break;
        case "connecting":
          indicator.classList.add("bg-yellow-400");
          break;
      }

      statusText.textContent = text;
    }
  }

  showToast(message, type = "info") {
    const container = document.getElementById("toast-container");
    if (!container) return;

    const colors = {
      success: "bg-green-500",
      error: "bg-red-500",
      warning: "bg-yellow-500",
      info: "bg-blue-500",
    };

    const icons = {
      success: "fa-check-circle",
      error: "fa-exclamation-circle",
      warning: "fa-exclamation-triangle",
      info: "fa-info-circle",
    };

    const toast = document.createElement("div");
    toast.className = `${colors[type]} text-white px-4 py-3 rounded-lg shadow-lg mb-2 max-w-sm transform translate-x-full transition-transform duration-300`;
    toast.innerHTML = `
      <div class="flex items-center">
        <i class="fas ${icons[type]} mr-2"></i>
        <span class="text-sm">${message}</span>
        <button class="ml-2 text-white hover:text-gray-200" onclick="this.parentElement.parentElement.remove()">
          <i class="fas fa-times text-xs"></i>
        </button>
      </div>
    `;

    container.appendChild(toast);

    // Animate in
    setTimeout(() => {
      toast.classList.remove("translate-x-full");
    }, 100);

    // Auto remove after 4 seconds
    setTimeout(() => {
      toast.classList.add("translate-x-full");
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
}

// Initialize application when DOM is loaded
document.addEventListener("DOMContentLoaded", function () {
  const app = new ColorDetectionApp();

  // Auto-start camera after 1 second
  setTimeout(() => {
    app.startCamera();
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
});
