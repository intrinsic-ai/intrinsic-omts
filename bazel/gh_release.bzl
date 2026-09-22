"""Bazel repository rule and module extension to download GitHub release assets."""

def _gh_release_file_impl(rctx):
    repo = rctx.attr.repo
    tag = rctx.attr.tag
    pattern = rctx.attr.pattern
    filename = rctx.attr.output if rctx.attr.output else pattern
    url = "https://github.com/{repo}/releases/download/{tag}/{pattern}".format(
        pattern = pattern,
        repo = repo,
        tag = tag,
    )

    if rctx.attr.archive:
        rctx.download_and_extract(
            url = [url],
            sha256 = rctx.attr.sha256,
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
        rctx.download(
            url = [url],
            output = filename,
            sha256 = rctx.attr.sha256,
        )
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
