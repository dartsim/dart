Section: Breaking Changes

  * DART log format strings support only `{}`, `{:.Ne}`, `{:.Nf}`, `{:.Ng}`
    (a decimal precision `N` from 0 to 1000 for floating-point values), and
    escaped braces `{{` and `}}`. Unsupported fields, invalid precision or
    missing arguments produce a format-error marker followed by the raw
    format string; surplus arguments are ignored. Default `long double`
    formatting preserves round trips but may differ in layout from fmt.
    [#3854](https://github.com/dartsim/dart/pull/3854)
