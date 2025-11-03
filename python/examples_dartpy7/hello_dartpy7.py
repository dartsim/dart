#!/usr/bin/env python3
"""Simple example using dartpy7 Python bindings."""

import dartpy7 as d


def main():
    print("DART 7.0 Python Example")
    print("=" * 40)

    # Print version info
    print(f"\nVersion: {d.__version__}")
    print(f"Major: {d.version_major()}")
    print(f"Minor: {d.version_minor()}")
    print(f"Patch: {d.version_patch()}")

    # Create a world
    print("\nCreating World...")
    world = d.World()
    print("World created successfully!")

    print("\nExample completed successfully!")


if __name__ == "__main__":
    main()
