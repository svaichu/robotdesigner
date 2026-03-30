
import importlib.resources as resources

def default_xml_path(filename: str) -> str:
	"""
	Resolve the default path to the bundled one_arm MuJoCo XML.

	Prefer package data (works when installed from wheel). If not available,
	fall back to the repo-relative assets folder for dev.
	"""
	# Try importlib.resources (Python 3.9+)
	try:
		with resources.as_file(resources.files("assets").joinpath(f"{filename}")) as p:
			if p.exists():
				return str(p)
	except Exception:
			raise FileNotFoundError(f"MuJoCo XML not found in package resources: assets/{filename}")
