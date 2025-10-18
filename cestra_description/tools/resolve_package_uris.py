#!/usr/bin/env python3
"""
resolve_package_uris.py

Simple helper: convert package://<pkg>/path URIs in a URDF file to absolute filesystem paths
so editors/visualizers that don't know ROS_PACKAGE_PATH can still load meshes.

Usage: python3 resolve_package_uris.py input.urdf output.urdf

It assumes this repository is checked out and the package folders are under the workspace src.
If a package isn't found locally, the original URI is left unchanged and a warning is printed.
"""
import os
import re
import sys

PKG_URI_RE = re.compile(r'package://([A-Za-z0-9_\-]+)/([^"\s]+)')


def find_package_path(workspace_src, pkg_name):
    """Return absolute path to package folder under workspace src or None."""
    candidate = os.path.join(workspace_src, pkg_name)
    if os.path.isdir(candidate):
        return candidate
    return None


def resolve_line(workspace_src, line):
    def repl(m):
        pkg = m.group(1)
        rel = m.group(2)
        pkg_path = find_package_path(workspace_src, pkg)
        if pkg_path:
            abs_path = os.path.join(pkg_path, rel)
            abs_path = os.path.abspath(abs_path)
            if os.path.exists(abs_path):
                return abs_path
            else:
                print(f"Warning: resolved path does not exist: {abs_path}")
                return abs_path
        else:
            print(f"Warning: package '{pkg}' not found under {workspace_src}")
            return m.group(0)

    return PKG_URI_RE.sub(repl, line)


def main():
    if len(sys.argv) < 3:
        print("Usage: resolve_package_uris.py input.urdf output.urdf")
        sys.exit(2)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    # try to locate workspace src by climbing up from this script location
    # assume script lives in <workspace>/src/cestra/cestra_description/tools
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # workspace src likely at ../../..
    workspace_src = os.path.normpath(os.path.join(script_dir, '..'))
    # If that folder contains package directories (e.g., cestra_description), use it; else use parent
    if not os.path.isdir(workspace_src):
        # fallback to current working directory + src
        workspace_src = os.path.join(os.getcwd(), 'src')

    # If given a path that looks like .../src/cestra/cestra_description/tools, we want src parent
    # attempt to find the nearest 'src' ancestor
    p = script_dir
    while p and p != os.path.dirname(p):
        if os.path.basename(p) == 'src':
            workspace_src = p
            break
        p = os.path.dirname(p)

    if not os.path.isdir(workspace_src):
        # try repo root/../src
        workspace_src = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))

    # Finally, if workspace_src still doesn't exist, take script parent
    if not os.path.isdir(workspace_src):
        workspace_src = os.path.abspath(os.path.join(script_dir, '..'))

    # If the resolved workspace_src ends with 'cestra_description', we want its parent (src)
    if workspace_src.endswith('cestra_description'):
        workspace_src = os.path.dirname(workspace_src)

    # If workspace_src is the package root, we want its parent 'src'
    if os.path.basename(workspace_src) != 'src':
        # try to locate a 'src' sibling
        cand = os.path.join(os.path.dirname(workspace_src), 'src')
        if os.path.isdir(cand):
            workspace_src = cand

    print(f"Using workspace src: {workspace_src}")

    with open(input_file, 'r', encoding='utf-8') as fh_in, open(output_file, 'w', encoding='utf-8') as fh_out:
        for line in fh_in:
            fh_out.write(resolve_line(workspace_src, line))


if __name__ == '__main__':
    main()
