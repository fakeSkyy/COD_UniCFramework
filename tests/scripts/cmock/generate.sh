#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TESTS_ROOT=$(CDPATH= cd -- "${SCRIPT_DIR}/../.." && pwd)
REPO_ROOT=$(CDPATH= cd -- "${TESTS_ROOT}/.." && pwd)
DOMAINS="$SCRIPT_DIR/domains.tsv"
domain=
output=

while [ "$#" -gt 0 ]
do
    case "$1" in
        --domain)
            [ "$#" -ge 2 ] || { echo "--domain requires a value" >&2; exit 2; }
            domain=$2
            shift 2
            ;;
        --output)
            [ "$#" -ge 2 ] || { echo "--output requires a value" >&2; exit 2; }
            output=$2
            shift 2
            ;;
        -h | --help)
            echo "Usage: $0 --domain NAME --output DIRECTORY"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 2
            ;;
    esac
done

[ -n "$domain" ] || { echo "--domain is required" >&2; exit 2; }
[ -n "$output" ] || { echo "--output is required" >&2; exit 2; }
record=$(awk -F '\t' -v wanted="$domain" 'NR > 1 && $1 == wanted { print; found = 1 } END { if (!found) exit 1 }' "$DOMAINS") || {
    echo "Unknown CMock domain: $domain" >&2
    exit 2
}
tab=$(printf '\t')
IFS=$tab read -r domain config committed_output driver inputs <<EOF
$record
EOF

case "$output" in
    /*) ;;
    *) output=$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$output") ;;
esac
case "$output" in
    "$REPO_ROOT" | "$REPO_ROOT"/*)
        echo "CMock output must be a disposable directory outside the repository: $output" >&2
        exit 2
        ;;
esac
mkdir -p "$output"

if [ -n "${RUBY:-}" ]
then
    ruby_bin=$RUBY
elif [ -n "${CMOCK_RUBY_ROOT:-}" ]
then
    ruby_bin="${CMOCK_RUBY_ROOT}/usr/bin/ruby3.2"
elif command -v ruby3.2 >/dev/null 2>&1
then
    ruby_bin=ruby3.2
else
    ruby_bin=ruby
fi
if ! command -v "$ruby_bin" >/dev/null 2>&1 && [ ! -x "$ruby_bin" ]
then
    echo "Ruby is required only to regenerate CMock files; committed mocks can still be built." >&2
    exit 1
fi

if [ -n "${CMOCK_RUBY_ROOT:-}" ]
then
    RUBYLIB="${CMOCK_RUBY_ROOT}/usr/lib/ruby/3.2.0:${CMOCK_RUBY_ROOT}/usr/lib/x86_64-linux-gnu/ruby/3.2.0:${CMOCK_RUBY_ROOT}/usr/lib/ruby/vendor_ruby${RUBYLIB:+:$RUBYLIB}"
    LD_LIBRARY_PATH="${CMOCK_RUBY_ROOT}/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export RUBYLIB LD_LIBRARY_PATH
fi

work_dir=$(mktemp -d "${TMPDIR:-/tmp}/cod-cmock-generate.XXXXXX")
cleanup()
{
    rm -rf "$work_dir"
}
trap cleanup EXIT HUP INT TERM

cd "$REPO_ROOT"
case "$driver" in
    standard)
        generated_config="$work_dir/cmock.yml"
        awk -v path="$output" '
            /^[[:space:]]*:mock_path:/ { print "  :mock_path: " path; replaced = 1; next }
            { print }
            END { if (!replaced) exit 1 }
        ' "$config" > "$generated_config"
        old_ifs=$IFS
        IFS=';'
        set -- $inputs
        IFS=$old_ifs
        "$ruby_bin" tests/cmock/lib/cmock.rb -o "$generated_config" "$@"
        ;;
    f4-inline)
        CMOCK_OUTPUT=$output CMOCK_INPUTS=$inputs \
            "$ruby_bin" -I tests/cmock/lib <<'RUBY'
require 'cmock'

options = {
  mock_path: ENV.fetch('CMOCK_OUTPUT'),
  mock_prefix: 'mock_',
  verbosity: 2,
  enforce_strict_ordering: true,
  plugins: %i[ignore ignore_arg expect_any_args callback return_thru_ptr],
  when_no_prototypes: :warn,
  treat_externs: :include,
  strippables: ['(?:__attribute__\\s*\\(\\([^)]*\\)\\))']
}

CMock.new(options).setup_mocks(ENV.fetch('CMOCK_INPUTS').split(';'))
RUBY
        ;;
    integration)
        old_ifs=$IFS
        IFS=';'
        set -- $inputs
        IFS=$old_ifs
        for input in "$@"
        do
            relative=${input#tests/integration/}
            target=${relative%%/*}
            target_output="$output/$target/mocks"
            mkdir -p "$target_output"
            generated_config="$work_dir/integration-$target.yml"
            awk -v path="$target_output" '
                { print }
                /^:cmock:$/ { print "  :mock_path: " path }
            ' "$config" > "$generated_config"
            "$ruby_bin" tests/cmock/lib/cmock.rb -o "$generated_config" "$input"
        done
        ;;
    *)
        echo "Unsupported CMock driver: $driver" >&2
        exit 2
        ;;
esac
