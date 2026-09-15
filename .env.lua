-- maptrax's directory environment. Loaded when you `cd` here, unloaded when you leave.

oslo.direnv.nix_develop()

-- Make the flake python (with rerun-sdk etc.) win over any system python.
local python = os.getenv("PYTHON")
if python and python ~= "" then
  oslo.direnv.path_add(python:match("^(.*)/[^/]+$") or ".")
  -- `make bind-py` installs the maptrax extension wheel here.
  oslo.direnv.path_add(oslo.sys.pwd() .. "/target/python", "PYTHONPATH")
end

oslo.direnv.path_add("./target/debug")
oslo.direnv.path_add("./target/release")

oslo.env.set("TOP_HEAD", oslo.sys.pwd())

-- Shared with every other robolibs checkout so the Wayland/NVIDIA detection lives in one place.
oslo.source("/home/bresilla/data/code/robolibs/.display.sh")

oslo.env.set_alias("_b", "make build")
oslo.env.set_alias("_c", "make compile")
oslo.env.set_alias("_r", "make run")
oslo.env.set_alias("_t", "make test")
