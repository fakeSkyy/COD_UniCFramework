#!/usr/bin/env sh
set -u

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
json_out=
update=0
reference_root=$REPO_ROOT

while [ "$#" -gt 0 ]
do
    case "$1" in
        --json-out)
            [ "$#" -ge 2 ] || { echo "--json-out requires a value" >&2; exit 2; }
            json_out=$2
            shift 2
            ;;
        --update)
            update=1
            shift
            ;;
        --reference-root)
            [ "$#" -ge 2 ] || { echo "--reference-root requires a value" >&2; exit 2; }
            reference_root=$2
            shift 2
            ;;
        -h | --help)
            echo "Usage: $0 [--json-out FILE] [--update] [--reference-root DIRECTORY]"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

if [ "$update" -eq 1 ] && [ "$reference_root" != "$REPO_ROOT" ]
then
    echo "--update cannot be combined with --reference-root" >&2
    exit 2
fi

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/cod-cmock-check.XXXXXX")
cleanup()
{
    rm -rf "$work_dir"
}
trap cleanup EXIT HUP INT TERM

generation_failures="$work_dir/generation-failures.txt"
: > "$generation_failures"
tab=$(printf '\t')
while IFS=$tab read -r domain config output driver inputs
do
    [ "$domain" != "domain" ] || continue
    printf 'CMock %-22s generate ... ' "$domain"
    if "$SCRIPT_DIR/generate.sh" --domain "$domain" --output "$work_dir/generated/$domain" \
        > "$work_dir/$domain.log" 2>&1
    then
        printf 'done\n'
    else
        result=$?
        printf 'FAIL (%s)\n' "$result" >&2
        printf '%s\t%s\n' "$domain" "$result" >> "$generation_failures"
        cat "$work_dir/$domain.log" >&2
    fi
done < "$SCRIPT_DIR/domains.tsv"

python3 - "$SCRIPT_DIR/domains.tsv" "$work_dir/generated" "$reference_root" \
    "$generation_failures" "$json_out" "$update" <<'PY'
import csv
import hashlib
import json
import pathlib
import shutil
import sys

manifest, generated_root, reference_root, failures_path, json_out, update_arg = sys.argv[1:]
generated_root = pathlib.Path(generated_root)
reference_root = pathlib.Path(reference_root)
update = update_arg == "1"
generation_failures = {}
for line in pathlib.Path(failures_path).read_text(encoding="utf-8").splitlines():
    if line:
        domain, rc = line.split("\t", 1)
        generation_failures[domain] = int(rc)

with pathlib.Path(manifest).open(encoding="utf-8", newline="") as stream:
    domains = list(csv.DictReader(stream, delimiter="\t"))

def files_under(root: pathlib.Path, integration: bool) -> dict[str, pathlib.Path]:
    if not root.is_dir():
        return {}
    result = {}
    for path in root.rglob("*"):
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if integration and "/mocks/" not in f"/{relative}":
            continue
        result[relative] = path
    return result

def digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

results = []
passed = True
for entry in domains:
    domain = entry["domain"]
    committed = reference_root / entry["output"]
    generated = generated_root / domain
    integration = entry["driver"] == "integration"
    if domain in generation_failures:
        result = {
            "domain": domain,
            "status": "FAIL",
            "generation_exit_code": generation_failures[domain],
            "missing": [], "extra": [], "different": [],
        }
        passed = False
        results.append(result)
        continue

    if update:
        if integration:
            for old in committed.glob("*/mocks"):
                shutil.rmtree(old)
            for source in generated.glob("*/mocks"):
                destination = committed / source.relative_to(generated)
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copytree(source, destination)
        else:
            shutil.rmtree(committed, ignore_errors=True)
            committed.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(generated, committed)

    expected = files_under(generated, integration)
    actual = files_under(committed, integration)
    missing = sorted(set(expected) - set(actual))
    extra = sorted(set(actual) - set(expected))
    different = sorted(
        name for name in set(expected) & set(actual)
        if digest(expected[name]) != digest(actual[name])
    )
    ok = not (missing or extra or different)
    passed = passed and ok
    result = {
        "domain": domain,
        "status": "PASS" if ok else "FAIL",
        "missing": missing,
        "extra": extra,
        "different": different,
    }
    results.append(result)
    print(f"CMock {domain:<22} {'PASS' if ok else 'FAIL'}")
    for kind in ("missing", "extra", "different"):
        for name in result[kind]:
            print(f"  {kind}: {name}")

report = {"passed": passed, "updated": update, "domains": results}
if json_out:
    destination = pathlib.Path(json_out)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    temporary.replace(destination)
raise SystemExit(0 if passed else 1)
PY
