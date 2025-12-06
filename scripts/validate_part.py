#!/usr/bin/env python3
"""
Validation script for Part-level content.
Checks:
- Chapter completeness (all chapters present)
- Navigation integrity (_category_.json files)
- Lesson numbering consistency
- Content consistency across chapters
"""

import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List
import re


class PartValidator:
    """Validates a complete Part directory structure and content."""
    
    def __init__(self, part_dir: Path):
        """Initialize validator with Part directory path."""
        self.part_dir = part_dir
        self.errors: List[str] = []
        self.warnings: List[str] = []
        
    def validate(self) -> bool:
        """Run all validation checks. Returns True if all checks pass."""
        if not self.part_dir.exists() or not self.part_dir.is_dir():
            self.errors.append(f"Part directory not found: {self.part_dir}")
            return False
        
        self._validate_part_category()
        self._validate_chapters()
        self._validate_lesson_numbering()
        self._validate_navigation_links()
        
        return len(self.errors) == 0
    
    def _validate_part_category(self):
        """Check Part-level _category_.json exists and is valid."""
        category_file = self.part_dir / "_category_.json"
        if not category_file.exists():
            self.errors.append(f"Missing _category_.json in {self.part_dir.name}")
            return
        
        try:
            with open(category_file, 'r', encoding='utf-8') as f:
                category = json.load(f)
            
            # Check required fields
            if "label" not in category:
                self.errors.append(f"Missing 'label' in {category_file}")
            
            if "position" not in category:
                self.errors.append(f"Missing 'position' in {category_file}")
            
        except json.JSONDecodeError as e:
            self.errors.append(f"Invalid JSON in {category_file}: {e}")
    
    def _validate_chapters(self):
        """Check that all chapter directories are properly structured."""
        chapter_dirs = sorted([d for d in self.part_dir.iterdir() if d.is_dir()])
        
        if not chapter_dirs:
            self.errors.append(f"No chapter directories found in {self.part_dir.name}")
            return
        
        for chapter_dir in chapter_dirs:
            # Check for _category_.json
            category_file = chapter_dir / "_category_.json"
            if not category_file.exists():
                self.errors.append(
                    f"Missing _category_.json in {chapter_dir.name}"
                )
            else:
                try:
                    with open(category_file, 'r', encoding='utf-8') as f:
                        category = json.load(f)
                    
                    if "label" not in category:
                        self.errors.append(
                            f"Missing 'label' in {chapter_dir.name}/_category_.json"
                        )
                except json.JSONDecodeError as e:
                    self.errors.append(
                        f"Invalid JSON in {chapter_dir.name}/_category_.json: {e}"
                    )
            
            # Check for index.md
            index_file = chapter_dir / "index.md"
            if not index_file.exists():
                self.warnings.append(
                    f"Missing index.md in {chapter_dir.name}"
                )
            
            # Check for lesson files
            lesson_files = sorted([
                f for f in chapter_dir.glob("*.md")
                if f.name != "index.md" and re.match(r'^\d{2}-.+\.md$', f.name)
            ])
            
            if not lesson_files:
                self.errors.append(
                    f"No lesson files found in {chapter_dir.name}"
                )
    
    def _validate_lesson_numbering(self):
        """Check that lesson numbering is consistent within each chapter."""
        chapter_dirs = sorted([d for d in self.part_dir.iterdir() if d.is_dir()])
        
        for chapter_dir in chapter_dirs:
            lesson_files = sorted([
                f for f in chapter_dir.glob("*.md")
                if f.name != "index.md" and re.match(r'^\d{2}-.+\.md$', f.name)
            ])
            
            expected_number = 1
            for lesson_file in lesson_files:
                match = re.match(r'^(\d{2})-.+\.md$', lesson_file.name)
                if match:
                    actual_number = int(match.group(1))
                    if actual_number != expected_number:
                        self.warnings.append(
                            f"Lesson numbering gap in {chapter_dir.name}: "
                            f"expected {expected_number:02d}, found {actual_number:02d} "
                            f"({lesson_file.name})"
                        )
                    expected_number = actual_number + 1
    
    def _validate_navigation_links(self):
        """Check that navigation links between lessons are consistent."""
        chapter_dirs = sorted([d for d in self.part_dir.iterdir() if d.is_dir()])
        
        for chapter_dir in chapter_dirs:
            lesson_files = sorted([
                f for f in chapter_dir.glob("*.md")
                if f.name != "index.md" and re.match(r'^\d{2}-.+\.md$', f.name)
            ])
            
            for i, lesson_file in enumerate(lesson_files):
                try:
                    content = lesson_file.read_text(encoding='utf-8')
                    
                    # Check if "What's Next" section exists
                    if "What's Next" in content:
                        # If not the last lesson, should have a link
                        if i < len(lesson_files) - 1:
                            next_lesson = lesson_files[i + 1]
                            # Look for link to next lesson
                            if next_lesson.stem not in content:
                                self.warnings.append(
                                    f"{lesson_file.name}: 'What's Next' may not link to next lesson"
                                )
                
                except Exception as e:
                    self.warnings.append(
                        f"Error reading {lesson_file.name}: {e}"
                    )
    
    def print_report(self):
        """Print validation report to stdout."""
        print(f"\n{'='*70}")
        print(f"Part Validation Report: {self.part_dir.name}")
        print(f"{'='*70}\n")
        
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
            print("✅ All checks passed!")
        elif not self.errors:
            print("✅ All critical checks passed (warnings present)")
        else:
            print("❌ Validation failed")
        
        print()


def main():
    """Main entry point for command-line usage."""
    parser = argparse.ArgumentParser(
        description="Validate Part-level content structure"
    )
    parser.add_argument(
        "part_dir",
        type=Path,
        help="Path to the Part directory (e.g., docs/part-01-foundations/)"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as errors"
    )
    
    args = parser.parse_args()
    
    validator = PartValidator(args.part_dir)
    is_valid = validator.validate()
    validator.print_report()
    
    # Exit with error code if validation failed
    if not is_valid or (args.strict and validator.warnings):
        sys.exit(1)
    
    sys.exit(0)


if __name__ == "__main__":
    main()
