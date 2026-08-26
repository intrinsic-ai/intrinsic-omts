"""Bazel repository rule and module extension to download GitHub release assets using the gh CLI."""

def _gh_release_file_impl(rctx):
    repo = rctx.attr.repo
    tag = rctx.attr.tag
    pattern = rctx.attr.pattern
    filename = rctx.attr.output if rctx.attr.output else pattern

    gh_path = rctx.which("gh")
    if not gh_path:
        fail("The 'gh' binary was not found in PATH. Please install or authenticate GitHub CLI.")

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
    )
    if res.return_code != 0:
        fail("Failed to download GitHub release asset: %s\n%s" % (res.stdout, res.stderr))

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
            )

_download_tag = tag_class(
    attrs = {
        "name": attr.string(mandatory = True),
        "repo": attr.string(mandatory = True),
        "tag": attr.string(mandatory = True),
        "pattern": attr.string(mandatory = True),
        "output": attr.string(),
    },
)

gh_release = module_extension(
    implementation = _gh_release_extension_impl,
    tag_classes = {
        "download": _download_tag,
    },
)
