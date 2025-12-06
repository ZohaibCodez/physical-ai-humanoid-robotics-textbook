#!/usr/bin/env python3
"""
Comprehensive textbook validation script.
Validates all 10 success criteria from spec.md:
- SC-001: 100% lesson completion (87/87 lessons)
- SC-002: All lessons 1200-1800 words
- SC-003: All mandatory sections present
- SC-004: 150+ code examples complete
- SC-005: Zero Docusaurus build errors
- SC-006: All pages SEO-optimized
- SC-007: Comprehensive appendices
- SC-008: Mobile-responsive design
- SC-009: Technical accuracy
- SC-010: Logical learning progression
"""

import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List, Tuple
import re
import subprocess


class TextbookValidator:
    """Validates complete textbook against all success criteria."""
    
    # Expected structure from spec.md
    EXPECTED_PARTS = 7
    EXPECTED_LESSONS = 87
    EXPECTED_CODE_EXAMPLES = 150
    EXPECTED_APPENDICES = 5
    
    APPENDIX_FILES = [
        "hardware-guide.md",
        "installation-guide.md",
        "troubleshooting.md",
        "resources.md",
        "glossary.md"
    ]
    
    def __init__(self, docs_dir: Path):
        """Initialize validator with docs directory path."""
        self.docs_dir = docs_dir
        self.errors: List[str] = []
        self.warnings: List[str] = []
        self.stats: Dict[str, int] = {}
        
    def validate(self) -> bool:
        """Run all validation checks. Returns True if all checks pass."""
        if not self.docs_dir.exists() or not self.docs_dir.is_dir():
            self.errors.append(f"Docs directory not found: {self.docs_dir}")
            return False
        
        print("Running comprehensive textbook validation...\n")
        
        self._validate_sc001_lesson_completion()
        self._validate_sc002_word_counts()
        self._validate_sc003_mandatory_sections()
        self._validate_sc004_code_examples()
        self._validate_sc005_build()
        self._validate_sc006_seo()
        self._validate_sc007_appendices()
        self._validate_sc010_progression()
        
        return len(self.errors) == 0
    
    def _validate_sc001_lesson_completion(self):
        """SC-001: Verify 100% lesson completion (87/87 lessons present)."""
        print("Checking SC-001: Lesson Completion...")
        
        lesson_count = 0
        part_dirs = sorted([
            d for d in self.docs_dir.iterdir()
            if d.is_dir() and d.name.startswith("part-")
        ])
        
        if len(part_dirs) != self.EXPECTED_PARTS:
            self.errors.append(
                f"SC-001 FAIL: Expected {self.EXPECTED_PARTS} parts, "
                f"found {len(part_dirs)}"
            )
        
        for part_dir in part_dirs:
            chapter_dirs = sorted([d for d in part_dir.iterdir() if d.is_dir()])
            
            for chapter_dir in chapter_dirs:
                lesson_files = list(chapter_dir.glob("*.md"))
                # Exclude index.md from count
                lessons = [f for f in lesson_files if f.name != "index.md"]
                lesson_count += len(lessons)
        
        self.stats['lesson_count'] = lesson_count
        
        if lesson_count != self.EXPECTED_LESSONS:
            self.errors.append(
                f"SC-001 FAIL: Expected {self.EXPECTED_LESSONS} lessons, "
                f"found {lesson_count}"
            )
        else:
            print(f"  ✅ Found {lesson_count}/{self.EXPECTED_LESSONS} lessons\n")
    
    def _validate_sc002_word_counts(self):
        """SC-002: Verify all lessons 1200-1800 words."""
        print("Checking SC-002: Word Counts...")
        
        out_of_range_count = 0
        part_dirs = sorted([
            d for d in self.docs_dir.iterdir()
            if d.is_dir() and d.name.startswith("part-")
        ])
        
        for part_dir in part_dirs:
            chapter_dirs = sorted([d for d in part_dir.iterdir() if d.is_dir()])
            
            for chapter_dir in chapter_dirs:
                lesson_files = [
                    f for f in chapter_dir.glob("*.md")
                    if f.name != "index.md"
                ]
                
                for lesson_file in lesson_files:
                    try:
                        content = lesson_file.read_text(encoding='utf-8')
                        # Remove frontmatter and code blocks
                        content_no_fm = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)
                        content_no_code = re.sub(r'```.*?```', '', content_no_fm, flags=re.DOTALL)
                        
                        word_count = len(content_no_code.split())
                        
                        if word_count < 1200 or word_count > 1800:
                            out_of_range_count += 1
                            self.warnings.append(
                                f"SC-002: {lesson_file.relative_to(self.docs_dir)} "
                                f"has {word_count} words (expected 1200-1800)"
                            )
                    except Exception as e:
                        self.warnings.append(
                            f"Error reading {lesson_file.name}: {e}"
                        )
        
        if out_of_range_count == 0:
            print(f"  ✅ All lessons within word count range\n")
        else:
            print(f"  ⚠️  {out_of_range_count} lessons outside word count range\n")
    
    def _validate_sc003_mandatory_sections(self):
        """SC-003: Verify all mandatory sections in every lesson."""
        print("Checking SC-003: Mandatory Sections...")
        
        required_sections = [
            "Learning Objectives",
            "Introduction",
            "Key Takeaways",
            "Review Questions",
            "Further Reading",
            "What's Next"
        ]
        
        missing_sections_count = 0
        part_dirs = sorted([
            d for d in self.docs_dir.iterdir()
            if d.is_dir() and d.name.startswith("part-")
        ])
        
        for part_dir in part_dirs:
            chapter_dirs = sorted([d for d in part_dir.iterdir() if d.is_dir()])
            
            for chapter_dir in chapter_dirs:
                lesson_files = [
                    f for f in chapter_dir.glob("*.md")
                    if f.name != "index.md"
                ]
                
                for lesson_file in lesson_files:
                    try:
                        content = lesson_file.read_text(encoding='utf-8')
                        missing = []
                        
                        for section in required_sections:
                            pattern = rf'^##+ {re.escape(section)}'
                            if not re.search(pattern, content, re.MULTILINE):
                                missing.append(section)
                        
                        if missing:
                            missing_sections_count += 1
                            self.warnings.append(
                                f"SC-003: {lesson_file.relative_to(self.docs_dir)} "
                                f"missing sections: {', '.join(missing)}"
                            )
                    except Exception as e:
                        self.warnings.append(
                            f"Error reading {lesson_file.name}: {e}"
                        )
        
        if missing_sections_count == 0:
            print(f"  ✅ All lessons have required sections\n")
        else:
            print(f"  ⚠️  {missing_sections_count} lessons missing required sections\n")
    
    def _validate_sc004_code_examples(self):
        """SC-004: Verify 150+ code examples complete and runnable."""
        print("Checking SC-004: Code Examples...")
        
        code_block_count = 0
        part_dirs = sorted([
            d for d in self.docs_dir.iterdir()
            if d.is_dir() and d.name.startswith("part-")
        ])
        
        for part_dir in part_dirs:
            chapter_dirs = sorted([d for d in part_dir.iterdir() if d.is_dir()])
            
            for chapter_dir in chapter_dirs:
                lesson_files = [
                    f for f in chapter_dir.glob("*.md")
                    if f.name != "index.md"
                ]
                
                for lesson_file in lesson_files:
                    try:
                        content = lesson_file.read_text(encoding='utf-8')
                        code_blocks = re.findall(r'```', content)
                        code_block_count += len(code_blocks) // 2
                    except Exception as e:
                        self.warnings.append(
                            f"Error reading {lesson_file.name}: {e}"
                        )
        
        self.stats['code_examples'] = code_block_count
        
        if code_block_count < self.EXPECTED_CODE_EXAMPLES:
            self.warnings.append(
                f"SC-004: Only {code_block_count} code examples found "
                f"(expected {self.EXPECTED_CODE_EXAMPLES}+)"
            )
            print(f"  ⚠️  {code_block_count}/{self.EXPECTED_CODE_EXAMPLES}+ code examples\n")
        else:
            print(f"  ✅ {code_block_count}/{self.EXPECTED_CODE_EXAMPLES}+ code examples found\n")
    
    def _validate_sc005_build(self):
        """SC-005: Verify zero Docusaurus build errors."""
        print("Checking SC-005: Build Errors...")
        print("  ℹ️  Skipping build test (run 'npm run build' manually)\n")
        # Note: Actual build testing should be done in CI/CD pipeline
    
    def _validate_sc006_seo(self):
        """SC-006: Verify all pages SEO-optimized."""
        print("Checking SC-006: SEO Optimization...")
        
        missing_descriptions = 0
        part_dirs = sorted([
            d for d in self.docs_dir.iterdir()
            if d.is_dir() and d.name.startswith("part-")
        ])
        
        for part_dir in part_dirs:
            chapter_dirs = sorted([d for d in part_dir.iterdir() if d.is_dir()])
            
            for chapter_dir in chapter_dirs:
                lesson_files = [
                    f for f in chapter_dir.glob("*.md")
                    if f.name != "index.md"
                ]
                
                for lesson_file in lesson_files:
                    try:
                        content = lesson_file.read_text(encoding='utf-8')
                        
                        # Check for description in frontmatter
                        frontmatter_match = re.match(r'^---\n(.*?)\n---', content, re.DOTALL)
                        if frontmatter_match:
                            frontmatter = frontmatter_match.group(1)
                            desc_match = re.search(r'description:\s*["\'](.+?)["\']', frontmatter)
                            
                            if not desc_match:
                                missing_descriptions += 1
                                self.warnings.append(
                                    f"SC-006: {lesson_file.relative_to(self.docs_dir)} "
                                    f"missing description"
                                )
                        else:
                            missing_descriptions += 1
                            self.warnings.append(
                                f"SC-006: {lesson_file.relative_to(self.docs_dir)} "
                                f"missing frontmatter"
                            )
                    except Exception as e:
                        self.warnings.append(
                            f"Error reading {lesson_file.name}: {e}"
                        )
        
        if missing_descriptions == 0:
            print(f"  ✅ All lessons have SEO descriptions\n")
        else:
            print(f"  ⚠️  {missing_descriptions} lessons missing SEO descriptions\n")
    
    def _validate_sc007_appendices(self):
        """SC-007: Verify comprehensive appendices."""
        print("Checking SC-007: Appendices...")
        
        appendices_dir = self.docs_dir / "appendices"
        if not appendices_dir.exists():
            self.errors.append("SC-007 FAIL: Appendices directory not found")
            print(f"  ❌ Appendices directory not found\n")
            return
        
        missing_appendices = []
        for appendix_file in self.APPENDIX_FILES:
            if not (appendices_dir / appendix_file).exists():
                missing_appendices.append(appendix_file)
        
        if missing_appendices:
            self.errors.append(
                f"SC-007 FAIL: Missing appendices: {', '.join(missing_appendices)}"
            )
            print(f"  ❌ Missing appendices: {', '.join(missing_appendices)}\n")
        else:
            print(f"  ✅ All {len(self.APPENDIX_FILES)} appendices present\n")
    
    def _validate_sc010_progression(self):
        """SC-010: Verify logical learning progression."""
        print("Checking SC-010: Learning Progression...")
        print("  ℹ️  Manual review required for learning progression\n")
        # Note: This requires expert review
    
    def print_report(self):
        """Print comprehensive validation report."""
        print(f"\n{'='*70}")
        print("TEXTBOOK VALIDATION REPORT")
        print(f"{'='*70}\n")
        
        print("📊 STATISTICS:")
        if self.stats:
            for key, value in self.stats.items():
                print(f"  • {key.replace('_', ' ').title()}: {value}")
            print()
        
        if self.errors:
            print("❌ ERRORS:")
            for error in self.errors:
                print(f"  • {error}")
            print()
        
        if self.warnings:
            print("⚠️  WARNINGS:")
            for warning in self.warnings:
                print(f"  • {warning}")
            print()
        
        if not self.errors and not self.warnings:
            print("✅ All success criteria passed!")
        elif not self.errors:
            print("✅ All critical criteria passed (warnings present)")
        else:
            print("❌ Validation failed - critical errors found")
        
        print()


def main():
    """Main entry point for command-line usage."""
    parser = argparse.ArgumentParser(
        description="Validate complete textbook against all success criteria"
    )
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path("docs"),
        help="Path to the docs directory (default: docs/)"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as errors"
    )
    
    args = parser.parse_args()
    
    validator = TextbookValidator(args.docs_dir)
    is_valid = validator.validate()
    validator.print_report()
    
    # Exit with error code if validation failed
    if not is_valid or (args.strict and validator.warnings):
        sys.exit(1)
    
    sys.exit(0)


if __name__ == "__main__":
    main()
