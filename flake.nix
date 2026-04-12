{
  description = "maptrax Rust library development shell";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    { nixpkgs, rust-overlay, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        overlays = [ (import rust-overlay) ];

        pkgs = import nixpkgs {
          inherit system overlays;
        };

        python = pkgs.python3;
        rustToolchain = pkgs.rust-bin.stable."1.94.1".default.override {
          extensions = [ "rust-src" "rustfmt" "clippy" ];
        };
      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            rustToolchain
            pkgs.clang
            pkgs.mold
            pkgs.pkg-config
            pkgs.stdenv.cc.cc.lib
            pkgs.zlib
            python
            python.pkgs.pip
            python.pkgs.virtualenv
            pkgs.maturin
          ];

          RUST_SRC_PATH = "${pkgs.rust.packages.stable.rustPlatform.rustLibSrc}";
          PYO3_PYTHON = "${python}/bin/python";
          MAPTRAX_RUSTC = "${rustToolchain}/bin/rustc";
          MAPTRAX_CARGO = "${rustToolchain}/bin/cargo";
          shellHook = ''
            export PATH=${rustToolchain}/bin:$PATH
            export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib pkgs.zlib ]}:$LD_LIBRARY_PATH
          '';
        };
      }
    );
}
