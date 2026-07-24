{
  description = "Enginner STM32 development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils }:
    utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };
	      arm-toolchain = pkgs.gcc-arm-embedded;
      in
      {
        devShells.default = pkgs.mkShellNoCC {
          # nativeBuildInputs is for tools that run on the build machine
          nativeBuildInputs = with pkgs; [
            gcc-arm-embedded
            openocd
            stlink
            dfu-util

            # Build tools
            cmake
            ninja
            gnumake

            # Language Server and Tools
            llvmPackages_19.clang-tools

            # Additional useful tools
            python3
            picocom
          ];

          env = {
            CC = "arm-none-eabi-gcc";
            CXX = "arm-none-eabi-g++";
            ARM_SYSROOT = "${arm-toolchain}/arm-none-eabi";
            CLANGD_FLAGS = "--query-driver=${arm-toolchain}/bin/arm-none-eabi-*";
          };

          shellHook = ''
              echo Enginner Nix Env Done.
            '';

        };
      });
}
