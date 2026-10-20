{
  description = "SO-101 arm — LeRobot + Feetech bus tooling";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

  outputs =
    { nixpkgs, ... }:
    let
      systems = [
        "aarch64-darwin"
        "x86_64-darwin"
        "aarch64-linux"
        "x86_64-linux"
      ];
      forAll = f: nixpkgs.lib.genAttrs systems (s: f nixpkgs.legacyPackages.${s});
    in
    {
      devShells = forAll (
        pkgs:
        let
          python = pkgs.python312;
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              python
              uv
              bun
              nodejs_24
              ffmpeg_7
              just
              jq
            ];

            # LeRobot ships as PyPI wheels (torch, opencv, feetech-servo-sdk),
            # so the Python side lives in a uv-managed .venv pinned by uv.lock
            # rather than in nixpkgs. The hook keeps that venv in sync with the
            # lock on every shell entry; first entry downloads ~400 MB of wheels.
            shellHook = ''
              export UV_PYTHON=${python}/bin/python3
              export UV_PYTHON_DOWNLOADS=never
              ROOT=$(git -C "$PWD" rev-parse --show-toplevel 2>/dev/null || echo "$PWD")
              if ! uv sync --locked --quiet --project "$ROOT"; then
                echo "› uv sync failed — run 'uv sync' in $ROOT to see why"
              fi
              if ! bun install --frozen-lockfile --silent --cwd "$ROOT/Software/so101"; then
                echo "› bun install failed — run 'bun install' in $ROOT/Software/so101 to see why"
              fi
              export VIRTUAL_ENV="$ROOT/.venv"
              export PATH="$VIRTUAL_ENV/bin:$ROOT/Software/so101/bin:$PATH"
              # TorchCodec (LeRobot's video codec) dlopens FFmpeg 7's libraries by name; the fallback path
              # lets it find the nix ones without touching anything that resolves its libraries normally.
              export DYLD_FALLBACK_LIBRARY_PATH="${pkgs.ffmpeg_7.lib}/lib''${DYLD_FALLBACK_LIBRARY_PATH:+:$DYLD_FALLBACK_LIBRARY_PATH}"
            '';
          };
        }
      );
    };
}
