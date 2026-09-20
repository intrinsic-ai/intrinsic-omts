# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Starlark rule to extract skill AssetInfo and manifest files without OCI or MLModel runfiles."""

load("@intrinsic-core//intrinsic/assets/build_defs:asset.bzl", "AssetInfo")

def _solution_manifest_only_impl(ctx):
    skill_asset_infos = [
        s[AssetInfo].asset_info
        for s in ctx.attr.skills
        if AssetInfo in s
    ]
    all_files = skill_asset_infos + ctx.files.extra_manifests
    return [
        DefaultInfo(
            files = depset(all_files),
            runfiles = ctx.runfiles(files = all_files),
        ),
    ]

solution_manifest_only = rule(
    implementation = _solution_manifest_only_impl,
    attrs = {
        "extra_manifests": attr.label_list(
            allow_files = True,
            doc = "Additional manifest or BUILD files to expose alongside the skill AssetInfo files.",
        ),
        "skills": attr.label_list(
            providers = [[AssetInfo]],
            doc = "Skill asset targets whose AssetInfo.asset_info (.asset_info.binpb) files are exposed.",
        ),
    },
    doc = "Extracts skill AssetInfo.asset_info (.asset_info.binpb) files without building MLModel or container bundles.",
)
