#!/usr/bin/env python3
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

"""Generates an SVG test badge and GitHub Step Summary from Bazel test.xml logs."""

import argparse
import glob
import os
import pathlib
import xml.etree.ElementTree as ET


def render_badge_svg(label: str, message: str, color: str) -> str:
  """Renders a Shields.io-style flat SVG badge with dynamic width."""
  # Approximate DejaVu Sans 11px character width + padding
  label_width = int(len(label) * 6.5) + 12
  msg_width = int(len(message) * 6.5) + 12
  total_width = label_width + msg_width
  label_x = (label_width / 2.0) * 10
  msg_x = (label_width + msg_width / 2.0) * 10
  label_text_len = (label_width - 10) * 10
  msg_text_len = (msg_width - 10) * 10

  return f"""<svg xmlns="http://www.w3.org/2000/svg" width="{total_width}" height="20" role="img" aria-label="{label}: {message}">
  <title>{label}: {message}</title>
  <linearGradient id="s" x2="0" y2="100%">
    <stop offset="0" stop-color="#bbb" stop-opacity=".1"/>
    <stop offset="1" stop-opacity=".1"/>
  </linearGradient>
  <clipPath id="r">
    <rect width="{total_width}" height="20" rx="3" fill="#fff"/>
  </clipPath>
  <g clip-path="url(#r)">
    <rect width="{label_width}" height="20" fill="#555"/>
    <rect x="{label_width}" width="{msg_width}" height="20" fill="{color}"/>
    <rect width="{total_width}" height="20" fill="url(#s)"/>
  </g>
  <g fill="#fff" text-anchor="middle" font-family="Verdana,Geneva,DejaVu Sans,sans-serif" text-rendering="geometricPrecision" font-size="110">
    <text aria-hidden="true" x="{label_x:.0f}" y="150" fill="#010101" fill-opacity=".3" transform="scale(.1)" textLength="{label_text_len}">{label}</text>
    <text x="{label_x:.0f}" y="140" transform="scale(.1)" fill="#fff" textLength="{label_text_len}">{label}</text>
    <text aria-hidden="true" x="{msg_x:.0f}" y="150" fill="#010101" fill-opacity=".3" transform="scale(.1)" textLength="{msg_text_len}">{message}</text>
    <text x="{msg_x:.0f}" y="140" transform="scale(.1)" fill="#fff" textLength="{msg_text_len}">{message}</text>
  </g>
</svg>
"""


def collect_test_results(
  testlogs_dir: str,
) -> tuple[list[tuple[str, int, int]], int, int]:
  """Parses JUnit XML files under bazel-testlogs/tests/**/test.xml."""
  pattern = os.path.join(testlogs_dir, "tests", "**", "test.xml")
  xml_paths = sorted(glob.glob(pattern, recursive=True))
  suites: list[tuple[str, int, int]] = []
  total_tests = 0
  total_failed = 0

  for path in xml_paths:
    rel_suite = pathlib.Path(path).parent.name
    root = ET.parse(path).getroot()
    tests = sum(int(el.attrib.get("tests", 0)) for el in root.iter("testsuite"))
    failures = sum(
      int(el.attrib.get("failures", 0)) + int(el.attrib.get("errors", 0))
      for el in root.iter("testsuite")
    )
    suites.append((rel_suite, tests, failures))
    total_tests += tests
    total_failed += failures

  return suites, total_tests, total_failed


def format_step_summary(
  suites: list[tuple[str, int, int]], total_tests: int, total_failed: int
) -> str:
  """Formats a GitHub Actions Markdown step summary table."""
  passed = total_tests - total_failed
  status_icon = "✅" if total_failed == 0 else "❌"
  lines = [
    f"## {status_icon} Hermetic Unit Test Summary",
    "",
    f"- **Test Suites:** {len(suites)}",
    f"- **Total Test Cases:** {total_tests} ({passed} passed, {total_failed} failed)",
    "",
    "| Test Suite | Test Cases | Status |",
    "| :--- | :---: | :---: |",
  ]
  for suite_name, tests, failures in suites:
    suite_status = "✅ Passed" if failures == 0 else f"❌ {failures} Failed"
    lines.append(f"| `{suite_name}` | {tests} | {suite_status} |")
  lines.append("")
  return "\n".join(lines)


def main() -> None:
  """CLI entrypoint for generating the SVG badge and step summary."""
  parser = argparse.ArgumentParser(
    description="Generate test badge SVG and GitHub Step Summary."
  )
  parser.add_argument(
    "--testlogs_dir",
    default="bazel-testlogs",
    help="Path to Bazel testlogs directory.",
  )
  parser.add_argument(
    "--output",
    required=True,
    help="Output path for the generated SVG badge.",
  )
  args = parser.parse_args()

  suites, total_tests, total_failed = collect_test_results(args.testlogs_dir)
  if total_failed == 0 and total_tests > 0:
    message = f"{total_tests} passed"
    color = "#4c1"
  elif total_tests == 0:
    message = "unknown"
    color = "#9f9f9f"
  else:
    message = f"{total_failed} failed"
    color = "#e05d44"

  svg = render_badge_svg("tests", message, color)
  output_path = pathlib.Path(args.output)
  output_path.parent.mkdir(parents=True, exist_ok=True)
  output_path.write_text(svg, encoding="utf-8")

  summary_md = format_step_summary(suites, total_tests, total_failed)
  step_summary_path = os.environ.get("GITHUB_STEP_SUMMARY")
  if step_summary_path:
    with open(step_summary_path, "a", encoding="utf-8") as f:
      f.write(summary_md)

  print(
    f"Generated badge '{message}' across {len(suites)} suites -> {output_path}"
  )


if __name__ == "__main__":
  main()
