#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/generate_user_guide_pdf.sh [--output FILE]

Converts docs/user_guide_ko.html into a Korean PDF user guide.
Default output: dist/cocelo-hd-slam-yaw_user-guide_ko.pdf
EOF
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
SOURCE_HTML="${REPO_DIR}/docs/user_guide_ko.html"
OUTPUT_PDF="${REPO_DIR}/dist/cocelo-hd-slam-yaw_user-guide_ko.pdf"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --help|-h) usage; exit 0 ;;
    --output) OUTPUT_PDF="${2:?--output requires a file}"; shift 2 ;;
    --output=*) OUTPUT_PDF="${1#--output=}"; shift ;;
    *) echo "error: unsupported option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -f "${SOURCE_HTML}" ]] || { echo "error: manual source missing: ${SOURCE_HTML}" >&2; exit 1; }

OUTPUT_DIR="$(dirname -- "${OUTPUT_PDF}")"
OUTPUT_NAME="$(basename -- "${OUTPUT_PDF}")"
TEMP_DIR="$(mktemp -d /tmp/cocelo-user-guide.XXXXXX)"
cleanup() {
  find "${TEMP_DIR}" -depth -delete 2>/dev/null || true
}
trap cleanup EXIT

GENERATED_PDF="${TEMP_DIR}/$(basename -- "${SOURCE_HTML%.html}").pdf"
mkdir -p "${OUTPUT_DIR}"
if command -v libreoffice >/dev/null; then
  libreoffice --headless --convert-to pdf --outdir "${TEMP_DIR}" "${SOURCE_HTML}" >/dev/null
else
  CHROME_BIN="$(command -v google-chrome || command -v chromium || true)"
  [[ -n "${CHROME_BIN}" ]] || {
    echo "error: LibreOffice, Google Chrome, or Chromium is required to create the PDF" >&2
    exit 1
  }
  "${CHROME_BIN}" --headless --disable-gpu --no-sandbox \
    --print-to-pdf="${GENERATED_PDF}" "file://${SOURCE_HTML}" >/dev/null 2>&1
fi
[[ -f "${GENERATED_PDF}" ]] || { echo "error: PDF renderer did not create a PDF" >&2; exit 1; }
install -m 0644 "${GENERATED_PDF}" "${OUTPUT_DIR}/${OUTPUT_NAME}"
echo "Created: ${OUTPUT_DIR}/${OUTPUT_NAME}"
