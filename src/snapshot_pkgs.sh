#!/usr/bin/env bash
# Stage the current files of every package that has its own git repo into
# the src repo, without touching the package repos.
#
# Git skips any folder that contains a .git, so `git add` can't stage these
# files. Instead the files are written straight into the src index
# (hash-object + update-index). The package list comes from the
# "/pkg/" lines in .gitignore, which also stop `git add .` from staging the
# packages as empty submodule pointers.
#
# Usage:  ./snapshot_pkgs.sh            stage all packages
#         ./snapshot_pkgs.sh pkg1 pkg2  stage only these packages
# Then:   git commit -m "..."
set -euo pipefail
cd "$(dirname "$0")"

# Package repos were copied from another machine; skip the ownership check
G=(git -c safe.directory=* -c core.quotepath=off)

if [ $# -gt 0 ]; then
  pkgs=("$@")
else
  mapfile -t pkgs < <(sed -n 's#^/\(.*\)/$#\1#p' .gitignore)
fi

for pkg in "${pkgs[@]}"; do
  if [ ! -e "$pkg/.git" ]; then
    echo "skip $pkg (no .git)"
    continue
  fi

  # Files the package repo considers part of the package: tracked plus
  # untracked, minus the package's own .gitignore (build output etc.)
  mapfile -t files < <("${G[@]}" -C "$pkg" ls-files -co --exclude-standard |
                       while IFS= read -r f; do [ -f "$pkg/$f" ] && echo "$f"; done)
  if [ ${#files[@]} -eq 0 ]; then
    echo "skip $pkg (no files)"
    continue
  fi

  # File mode from the package index (keeps 100755 / 120000), else 100644
  declare -A mode=()
  while IFS=$'\t' read -r meta f; do
    mode["$f"]=${meta%% *}
  done < <("${G[@]}" -C "$pkg" ls-files -s)

  # Hash working-copy contents into the src object store in one batch
  mapfile -t hashes < <(printf '%s\n' "${files[@]/#/$pkg/}" |
                        "${G[@]}" hash-object -w --stdin-paths)

  {
    for i in "${!files[@]}"; do
      f=${files[$i]}
      printf '%s %s\t%s\n' "${mode[$f]:-100644}" "${hashes[$i]}" "$pkg/$f"
    done
    # Drop entries for files that no longer exist in the package
    comm -23 <("${G[@]}" ls-files -- "$pkg/" | sort) \
             <(printf '%s\n' "${files[@]/#/$pkg/}" | sort) |
      while IFS= read -r gone; do
        printf '0 %040d\t%s\n' 0 "$gone"
      done
  } | "${G[@]}" update-index --index-info

  unset mode
  echo "staged $pkg (${#files[@]} files)"
done
