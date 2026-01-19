# Resume: Header Snake_Case Migration

## Quick Start

```bash
# 1. Check current state
git status
git branch --show-current  # Should be: feature/header-snake-case-migration
git log --oneline -5

# 2. Verify build works
pixi run build

# 3. Run tests
pixi run test

# 4. If all passes, commit and continue
```

## Current State (as of last session)

### Commits Made

1. `22eeb6d3075` - Phase 1: DART 7 headers (All.hpp, Export.hpp, Fwd.hpp) - 21 files
2. `665a099f082` - Phase 2a: dart/common structural renames - 40 files

### Uncommitted Changes

- dart/math structural renames (32 files)
- dart/collision structural renames (46 files)
- dart/constraint structural renames (23 files)
- dart/dynamics structural renames (96 files)
- dart/gui structural renames (38 files)
- dart/utils structural renames (24 files)
- dart/sensor, dart/simulation, dart/optimizer, dart/lcpsolver (5 files)

Total uncommitted: ~264 renamed files + include updates

### Build Status

- Last build: PASSED (383/383 targets)
- Tests: Need to run
- Lint: Need to run

## Next Steps

### If Build Passes

1. Run `pixi run test` to verify tests pass
2. Run `pixi run lint` to verify code style
3. Commit all remaining changes:

   ```bash
   git add -A
   git commit -m "refactor: rename remaining headers to snake_case (structural renames)

   Phase 2b-h of PascalCase → snake_case header migration.

   Modules renamed:
   - dart/math (32 files)
   - dart/collision (46 files)
   - dart/constraint (23 files)
   - dart/dynamics (96 files)
   - dart/gui (38 files)
   - dart/utils (24 files)
   - dart/sensor, dart/simulation, dart/optimizer, dart/lcpsolver (5 files)

   Updated all internal includes and CMakeLists.txt files.
   CMake will auto-generate PascalCase compatibility wrappers."
   ```

4. Push and update PR

### If Build Fails

Check for:

1. Missing include updates - run:
   ```bash
   grep -rE '#include.*[A-Z][a-z]+[A-Z].*\.hpp' dart/ --include="*.cpp" --include="*.hpp"
   ```
2. CMakeLists.txt not updated - check files list in CMakeLists.txt

### If Tests Fail

1. Check if test files need include updates
2. Run individual failing test to diagnose

## Scripts Used

### Rename Headers Script

Save to `/tmp/rename_headers.py`:

```python
#!/usr/bin/env python3
import re, subprocess, sys
from pathlib import Path

def pascal_to_snake(name):
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()

def is_multi_word_pascal(name):
    return name[0].isupper() and bool(re.search(r'[a-z][A-Z]', name))

def find_headers(directory):
    return sorted([p for p in Path(directory).rglob('*.hpp')
                   if is_multi_word_pascal(p.stem)])

def main():
    directory = sys.argv[1]
    dry_run = '--dry-run' in sys.argv
    headers = find_headers(directory)

    for old_path in headers:
        new_name = pascal_to_snake(old_path.stem) + '.hpp'
        new_path = old_path.parent / new_name
        if old_path != new_path:
            print(f"{old_path} -> {new_path}")
            if not dry_run:
                subprocess.run(['git', 'mv', str(old_path), str(new_path)])

if __name__ == '__main__':
    main()
```

### Update CMakeLists Script

Save to `/tmp/update_cmake.py`:

```python
#!/usr/bin/env python3
import subprocess, re
from pathlib import Path

def main():
    result = subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True)
    renames = {}
    for line in result.stdout.strip().split('\n'):
        if ' -> ' in line:
            parts = line.split(' -> ')
            if len(parts) == 2 and parts[1].endswith('.hpp'):
                old_name = parts[0].split()[-1].split('/')[-1]
                new_name = parts[1].split('/')[-1]
                if old_name != new_name:
                    renames[old_name] = new_name

    sorted_renames = sorted(renames.items(), key=lambda x: len(x[0]), reverse=True)

    for cmake_file in Path('dart').rglob('CMakeLists.txt'):
        content = cmake_file.read_text()
        original = content
        for old_name, new_name in sorted_renames:
            pattern = rf'([\s/])({re.escape(old_name)})([\s\n\r)$]|$)'
            content = re.sub(pattern, rf'\1{new_name}\3', content)
        if content != original:
            cmake_file.write_text(content)
            print(f"Updated: {cmake_file}")

if __name__ == '__main__':
    main()
```

## Reference

- Migration strategy: `docs/onboarding/header-migration-analysis.md`
- PR: https://github.com/dartsim/dart/pull/2475
- Branch: `feature/header-snake-case-migration`
