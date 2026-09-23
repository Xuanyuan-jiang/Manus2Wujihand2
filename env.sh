# source this (bash or zsh): activates conda env wuji2 and exports project paths
if [ -n "${ZSH_VERSION:-}" ]; then
  eval '_wuji2_src=${(%):-%x}'
else
  _wuji2_src=${BASH_SOURCE[0]}
fi
export WUJI2_PROJ="${WUJI2_PROJ:-$(cd "$(dirname "$_wuji2_src")" && pwd)}"
unset _wuji2_src
export WUJI2_CONDA="${WUJI2_CONDA:-$HOME/miniconda3}"
export WUJI2_ENV="${WUJI2_ENV:-wuji2}"
export WUJI2_PY="$WUJI2_CONDA/envs/$WUJI2_ENV/bin/python"
if [ ! -x "$WUJI2_PY" ]; then
  echo "env.sh: missing interpreter $WUJI2_PY" >&2
  return 1 2>/dev/null || exit 1
fi
# shellcheck disable=SC1091
. "$WUJI2_CONDA/etc/profile.d/conda.sh"
conda activate "$WUJI2_ENV"
echo "env.sh OK: $(python -V 2>&1)  prefix=$CONDA_PREFIX"
