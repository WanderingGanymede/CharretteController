{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  buildInputs = with pkgs; [
    # Core Toolchain
    cmake           # Build system generator
    gcc-arm-embedded # ARM GCC cross-compiler toolchain
    libusb1         # USB library for tools like picotool

    # Pico-Specific Tools
    pioasm          # Assembles PIO programs (available in nixpkgs) [citation:1][citation:8]

    # Helpful Utilities
    minicom         # Serial terminal for debugging output
    picotool        # For inspecting UF2 files and interacting with the Pico (optional, from nixpkgs)

    (python3.withPackages (ps: with ps; [ matplotlib numpy tornado pandas]))
  ];

  # Environment variables that many Pico projects expect
  shellHook = ''
    echo "🔧 Raspberry Pi Pico W Development Environment Ready"
    echo "📁 PICO_SDK_PATH must be set to your local clone of the pico-sdk."
    echo "   Example: export PICO_SDK_PATH=$HOME/pico/pico-sdk"
    echo ""
    echo "💡 Tip: Clone the SDK and examples from GitHub if you haven't already:"
    echo "   git clone https://github.com/raspberrypi/pico-sdk.git"
    echo "   git clone https://github.com/raspberrypi/pico-examples.git"
  '';
}
