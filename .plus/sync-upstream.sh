#!/usr/bin/env bash
# Mirrors CoMaps from Codeberg and merges it into main.
# A clean merge is pushed to main directly, then the Android build runs as a
# compile check. A conflicting merge opens an issue and leaves main alone.
set -euo pipefail

UPSTREAM_URL="${UPSTREAM_URL:-https://codeberg.org/comaps/comaps.git}"
UPSTREAM_API="${UPSTREAM_API:-https://codeberg.org/api/v1/repos/comaps/comaps}"
OLD_SYNC_BRANCH="sync/upstream"
: "${GITHUB_REPOSITORY:?GITHUB_REPOSITORY is not set}"  # gh reads it
# Paths whose changes can make prepared route files (.plusovl) give wrong routes.
ROUTING_PATHS=(libs/routing libs/routing_common)

git config user.name "comaps-plus-sync"
git config user.email "comaps-plus-sync@users.noreply.github.com"

git remote get-url codeberg >/dev/null 2>&1 || git remote add codeberg "$UPSTREAM_URL"
git fetch --no-tags codeberg main
git fetch codeberg '+refs/tags/*:refs/tags/upstream/*'

gh label create upstream-sync    --color 0e8a16 --description "Merge of upstream CoMaps"        2>/dev/null || true
gh label create upstream-release --color 1d76db --description "New upstream CoMaps release"     2>/dev/null || true
gh label create sync-conflict    --color d93f0b --description "Upstream merge needs manual work" 2>/dev/null || true
gh label create routing-change   --color fbca04 --description "Upstream changed routing code"    2>/dev/null || true

new_tip=$(git rev-parse codeberg/main)
old_tip=$(git rev-parse -q --verify refs/remotes/origin/upstream || true)
if [ "$new_tip" != "$old_tip" ]; then
  git push origin "$new_tip:refs/heads/upstream"
fi
git push origin 'refs/tags/upstream/*:refs/tags/upstream/*' \
  || echo "::warning::some upstream tags could not be pushed (a tag may have been moved upstream)"

short=$(git rev-parse --short codeberg/main)

# Pull requests and the branch from the old review-first flow.
open_pr=$(gh pr list --head "$OLD_SYNC_BRANCH" --state open --json number --jq '.[0].number // empty')
open_conflict=$(gh issue list --label sync-conflict --state open --json number --jq '.[0].number // empty')

cleanup_old_flow() {
  if [ -n "$open_pr" ]; then
    gh pr close "$open_pr" --delete-branch --comment "$1" || true
  elif git ls-remote --exit-code --heads origin "$OLD_SYNC_BRANCH" >/dev/null 2>&1; then
    git push origin --delete "$OLD_SYNC_BRANCH" || true
  fi
}

if git merge-base --is-ancestor codeberg/main origin/main; then
  echo "main already contains upstream ($short)"
  cleanup_old_flow "main already contains upstream."
  if [ -n "$open_conflict" ]; then
    gh issue close "$open_conflict" --comment "main already contains upstream."
  fi
else
  base=$(git merge-base origin/main codeberg/main)
  git checkout -q --detach origin/main
  if git merge --no-ff -q -m "Merge upstream CoMaps $short" codeberg/main; then
    ahead=$(git rev-list --count origin/main..codeberg/main)
    if ! git push origin HEAD:main; then
      echo "::error::could not push the merge to main (main moved, or the token cannot push workflow files); the next run tries again"
      exit 1
    fi
    echo "merged $ahead upstream commits ($short) into main"
    cleanup_old_flow "Merged into main automatically."
    if [ -n "$open_conflict" ]; then
      gh issue close "$open_conflict" --comment "Upstream merged cleanly into main."
    fi

    routing_files=$(git diff --name-only "$base" codeberg/main -- "${ROUTING_PATHS[@]}")
    if [ -n "$routing_files" ]; then
      gh issue create --label routing-change \
        --title "Upstream changed routing code ($short)" \
        --body "$(printf 'The upstream merge %s, now in main, changes routing code:\n\n%s\n\nIf it changes edge weights, turn penalties, speeds or restriction handling, prepared route files give wrong routes until rebuilt: raise `kRoutingDataVersion` in `libs/routing/plus_overlay/overlay_file.hpp`.\n' \
          "$short" "$(printf '%s\n' "$routing_files" | sed 's/^/- /')")"
    fi

    # Compile check and a fresh test APK. GITHUB_TOKEN may start workflow_dispatch runs.
    GH_TOKEN="${DISPATCH_TOKEN:-$GH_TOKEN}" gh workflow run android-check.yaml --ref main \
      || echo "::warning::could not start Android Check"
  else
    conflicts=$(git diff --name-only --diff-filter=U)
    git merge --abort
    title="Upstream sync blocked by merge conflicts"
    body=$(printf 'Merging CoMaps main (%s) into main conflicts in:\n\n%s\n\nResolve locally:\n\n    git fetch origin\n    git checkout main\n    git pull\n    git merge origin/upstream\n    # fix the files, then\n    git commit\n    git push\n\nThis issue closes by itself on the next sync after main contains upstream.\n' \
      "$short" "$(printf '%s\n' "$conflicts" | sed 's/^/- /')")
    if [ -n "$open_conflict" ]; then
      gh issue edit "$open_conflict" --body "$body"
    else
      gh issue create --label sync-conflict --title "$title" --body "$body"
    fi
    cleanup_old_flow "Closed: upstream is now merged automatically when it merges cleanly; see the conflict issue."
  fi
fi

git fetch -q origin main
git checkout -q --detach origin/main

releases=$(curl -fsS "$UPSTREAM_API/releases?limit=10") || {
  echo "::warning::could not read upstream releases"
  exit 0
}

while IFS=$'\t' read -r tag url pre; do
  [ -n "$tag" ] || continue
  title="Upstream release $tag"
  commit=$(git rev-parse -q --verify "refs/tags/upstream/$tag^{commit}" || true)
  [ -n "$commit" ] || continue
  existing=$(gh issue list --state all --limit 200 --json number,title \
    --jq ".[] | select(.title == \"$title\") | .number" | head -n1)
  [ -z "$existing" ] || continue
  kind=release
  [ "$pre" = "true" ] && kind="pre-release"
  if git merge-base --is-ancestor "$commit" origin/main; then
    state="It is already in main. Run the Release workflow to publish a Comaps++ build that includes it, then close this issue."
  else
    state="It is not in main yet because the upstream merge conflicts; see the sync-conflict issue."
  fi
  gh issue create --label upstream-release --title "$title" \
    --body "$(printf 'Upstream published %s (%s): %s\n\n%s\n' "$tag" "$kind" "$url" "$state")"
done < <(printf '%s' "$releases" | jq -r '.[] | select(.draft == false) | [.tag_name, .html_url, .prerelease] | @tsv')
