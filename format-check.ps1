Get-ChildItem -Recurse -Include "*.c", "*.h"  | Where-Object { $_.FullName -CNotlike '*build*' } |
ForEach-Object {
    &clang-format --dry-run --Werror $_
}