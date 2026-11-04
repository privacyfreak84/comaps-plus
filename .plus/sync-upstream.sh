#!/usr/bin/env bash
set -euo pipefail

UPSTREAM_URL="${UPSTREAM_URL:-https://codeberg.org/comaps/comaps.git}"
UPSTREAM_API="${UPSTREAM_API:-https://codeberg.org/api/v1/repos/comaps/comaps}"
SYNC_BRANCH="sync/upstream"
REPO="${GITHUB_REPOSITORY:?GITHUB_REPOSITORY is not set}"

git config user.name "comaps-plus-sync"
git config user.email "comaps-plus-sync@users.noreply.github.com"

git remote get-url codeberg >/dev/null 2>&1 || git remote add codeberg "$UPSTREAM_URL"
git fetch --no-tags codeberg main
git fetch codeberg '+refs/tags/*:refs/tags/upstream/*'

gh label create upstream-sync    --color 0e8a16 --description "Merge of upstream CoMaps"        2>/dev/null || true
gh label create upstream-release --color 1d76db --description "New upstream CoMaps release"     2>/dev/null || true
gh label create sync-conflict    --color d93f0b --description "Upstream merge needs manual work" 2>/dev/null || true

new_tip=$(git rev-parse codeberg/main)
old_tip=$(git rev-parse -q --verify refs/remotes/origin/upstream || true)
if [ "$new_tip" != "$old_tip" ]; then
  git push origin "$new_tip:refs/heads/upstream"
fi
git push origin 'refs/tags/upstream/*:refs/tags/upstream/*' \
  || echo "::warning::some upstream tags could not be pushed (a tag may have been moved upstream)"

short=$(git rev-parse --short codeberg/main)

open_pr=$(gh pr list --head "$SYNC_BRANCH" --state open --json number --jq '.[0].number // empty')
open_conflict=$(gh issue list --label sync-conflict --state open --json number --jq '.[0].number // empty')

if git merge-base --is-ancestor codeberg/main origin/main; then
  echo "main already contains upstream ($short)"
  if [ -n "$open_pr" ]; then
    gh pr close "$open_pr" --comment "main already contains upstream."
  fi
  if [ -n "$open_conflict" ]; then
    gh issue close "$open_conflict" --comment "main already contains upstream."
  fi
else
  git checkout -q -B "$SYNC_BRANCH" origin/main
  if git merge --no-ff -q -m "Merge upstream CoMaps $short" codeberg/main; then
    git push --force origin "$SYNC_BRANCH"
    ahead=$(git rev-list --count origin/main..codeberg/main)
    body=$(printf 'Merges CoMaps main (%s) into main. Upstream commits not in main yet: %s.\n\nMost recent:\n\n%s\n' \
      "$short" "$ahead" "$(git log --format='- %h %s' -n 30 origin/main..codeberg/main)")
    if [ -n "$open_pr" ]; then
      gh pr edit "$open_pr" --title "Merge upstream CoMaps $short" --body "$body"
    else
      gh pr create --base main --head "$SYNC_BRANCH" --label upstream-sync \
        --title "Merge upstream CoMaps $short" --body "$body"
    fi
    if [ -n "$open_conflict" ]; then
      gh issue close "$open_conflict" --comment "Merge is clean again, see the sync pull request."
    fi
  else
    conflicts=$(git diff --name-only --diff-filter=U)
    git merge --abort
    title="Upstream sync blocked by merge conflicts"
    body=$(printf 'Merging CoMaps main (%s) into main conflicts in:\n\n%s\n\nResolve locally:\n\n    git fetch origin\n    git checkout -b sync-manual origin/main\n    git merge origin/upstream\n' \
      "$short" "$(printf '%s\n' "$conflicts" | sed 's/^/- /')")
    if [ -n "$open_conflict" ]; then
      gh issue edit "$open_conflict" --body "$body"
    else
      gh issue create --label sync-conflict --title "$title" --body "$body"
    fi
    if [ -n "$open_pr" ]; then
      gh pr close "$open_pr" --comment "Closed: upstream no longer merges cleanly, see the conflict issue."
    fi
  fi
fi

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
  existing=$(gh issue list --state all --limit 200 --json number,title,state \
    --jq ".[] | select(.title == \"$title\") | \"\(.number) \(.state)\"" | head -n1)
  if git merge-base --is-ancestor "$commit" origin/main; then
    case "$existing" in
      *OPEN) gh issue close "${existing%% *}" --comment "$tag is in main." ;;
    esac
  elif [ -z "$existing" ]; then
    kind=release
    [ "$pre" = "true" ] && kind=pre-release
    gh issue create --label upstream-release --title "$title" \
      --body "$(printf 'Upstream published %s (%s): %s\n\nThis release is not in main yet. Review and merge the sync pull request, then cut a Comaps++ release.\n' "$tag" "$kind" "$url")"
  fi
done < <(printf '%s' "$releases" | jq -r '.[] | select(.draft == false) | [.tag_name, .html_url, .prerelease] | @tsv')
