{
  # Nixpkgs / NixOS version to use.
  inputs.nixpkgs.url = "nixpkgs/24.05";
  inputs.flake-utils.url = "github:numtide/flake-utils";
  outputs = { self, nixpkgs, flake-utils, ... }:
    let
      # System types to support.
      supportedSystems =
        [ "x86_64-linux" "x86_64-darwin" "aarch64-linux" "aarch64-darwin" ];

      # Helper function to generate an attrset '{ x86_64-linux = f "x86_64-linux"; ... }'.
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;

      # Nixpkgs instantiated for supported system types.
      nixpkgsFor = forAllSystems (system: import nixpkgs { inherit system; });
    in {
      # contains a mutually consistent set of packages for a full toolchain using nextpnr-xilinx.
      devShell = forAllSystems (system:
        let nixpkgs = nixpkgsFor.${system};
          in nixpkgs.mkShell {
          buildInputs = (with nixpkgsFor.${system}; [
              findutils
              gnused
              gnugrep
              coreutils
              gnumake
              python312
              python312Packages.boost 
              cmake
              gcc
              eigen
           ]);
 
          shellHook =
            nixpkgs.lib.concatStrings [
              "export CC=" nixpkgs.gcc "/bin/gcc\n"
              "export CXX=" nixpkgs.gcc "/bin/g++\n"
            ];
        }
      );
  };
}
