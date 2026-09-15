"""Bazel repository rule and module extension to download GitHub release assets using the gh CLI."""

# intrinsic-ai/intrinsic-omts is private, so `gh` has to be authenticated.
# Interactively that is `gh auth login`; on a GitHub Actions runner it is one of
# these variables. They are forwarded explicitly rather than relying on the
# repository rule inheriting the client environment.
_GH_TOKEN_VARS = ["GH_TOKEN", "GITHUB_TOKEN"]

def _gh_environment(rctx):
    """Returns the authentication environment to pass to the `gh` CLI.

    Args:
      rctx: the repository context.

    Returns:
      A dict holding the first GitHub token variable that is set, or an empty
      dict when none is, in which case `gh` falls back to its own config file.
    """
    for name in _GH_TOKEN_VARS:
        value = rctx.getenv(name)
        if value:
            return {name: value}
    return {}

def _gh_release_file_impl(rctx):
    repo = rctx.attr.repo
    tag = rctx.attr.tag
    pattern = rctx.attr.pattern
    filename = rctx.attr.output if rctx.attr.output else pattern

    gh_path = rctx.which("gh")
    if not gh_path:
        fail("The 'gh' binary was not found in PATH. Please install or authenticate GitHub CLI.")

    rctx.report_progress("Downloading %s from %s@%s" % (pattern, repo, tag))
    res = rctx.execute(
        [
            gh_path,
            "release",
            "download",
            tag,
            "--repo",
            repo,
            "--pattern",
            pattern,
            "--output",
            filename,
            "--clobber",
        ],
        environment = _gh_environment(rctx),
    )
    if res.return_code != 0:
        fail(("Failed to download asset '%s' of release %s@%s: %s\n%s\n" +
              "%s is private, so `gh` must be authenticated: run `gh auth login` " +
              "locally, or set GH_TOKEN in CI. An unauthenticated `gh` reports " +
              "'release not found' rather than a permission error.") % (
            pattern,
            repo,
            tag,
            res.stdout,
            res.stderr,
            repo,
        ))

    if rctx.attr.sha256:
        sh_res = rctx.execute(["sha256sum", filename])
        if sh_res.return_code != 0:
            fail("Failed to compute sha256 checksum of downloaded asset %s" % filename)
        actual_sha256 = sh_res.stdout[:64]
        if actual_sha256 != rctx.attr.sha256:
            fail("Checksum mismatch for %s: expected %s, got %s" % (filename, rctx.attr.sha256, actual_sha256))

    if rctx.attr.archive:
        rctx.extract(
            archive = filename,
            stripPrefix = rctx.attr.strip_prefix,
        )
        rctx.file(
            "BUILD.bazel",
            content = """package(default_visibility = ["//visibility:public"])

exports_files(glob(["**"]))

filegroup(
    name = "all_files",
    srcs = glob(["**"]),
)
""",
        )
    else:
        rctx.file(
            "BUILD.bazel",
            content = """package(default_visibility = ["//visibility:public"])

exports_files(["{filename}"])

filegroup(
    name = "file",
    srcs = ["{filename}"],
)
""".format(filename = filename),
        )

gh_release_file = repository_rule(
    implementation = _gh_release_file_impl,
    attrs = {
        "repo": attr.string(mandatory = True, doc = "Owner/repo (e.g. 'intrinsic-ai/intrinsic-omts')"),
        "tag": attr.string(mandatory = True, doc = "Release tag (e.g. 'v0.0.1')"),
        "pattern": attr.string(mandatory = True, doc = "Asset filename pattern (e.g. 'moveit_flowstate_ros_bridge.bundle.tar')"),
        "output": attr.string(doc = "Target filename in the downloaded repository"),
        "archive": attr.bool(default = False, doc = "Whether to unpack the downloaded asset"),
        "strip_prefix": attr.string(default = "", doc = "Directory prefix to strip when extracting"),
        "sha256": attr.string(default = "", doc = "Expected SHA-256 checksum of downloaded asset"),
    },
)

def _gh_release_extension_impl(mctx):
    for mod in mctx.modules:
        for download in mod.tags.download:
            gh_release_file(
                name = download.name,
                repo = download.repo,
                tag = download.tag,
                pattern = download.pattern,
                output = download.output,
                archive = download.archive,
                strip_prefix = download.strip_prefix,
                sha256 = download.sha256,
            )

_download_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
        "repo": attr.string(mandatory = True),
        "tag": attr.string(mandatory = True),
        "pattern": attr.string(mandatory = True),
        "output": attr.string(),
        "archive": attr.bool(default = False),
        "strip_prefix": attr.string(default = ""),
        "sha256": attr.string(default = ""),
    },
)

gh_release = module_extension(
    implementation = _gh_release_extension_impl,
    tag_classes = {
        "download": _download_tag,
    },
)
