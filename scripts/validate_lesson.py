#!/usr/bin/env python3
"""
Validation script for individual lesson files.
Checks:
- Word count (1200-1800 range)
- Required sections (8 mandatory sections)
- Code blocks (≥2 for technical lessons)
- Admonitions (≥2 for engagement)
- Frontmatter (description ≤160 chars)
- Next lesson links
"""

import sys
import re
import argparse
from pathlib import Path
from typing import Dict, List, Tuple


class LessonValidator:
    """Validates a single lesson file against quality standards."""
    
    REQUIRED_SECTIONS = [
        "Learning Objectives",
        "Introduction",
        "Key Takeaways",
        "Review Questions",
        "Further Reading",
        "What's Next"
    ]
    
    MIN_WORDS = 1200
    MAX_WORDS = 1800
    MIN_CODE_BLOCKS = 2
    MIN_ADMONITIONS = 2
    MAX_DESCRIPTION_LENGTH = 160
    
    def __init__(self, file_path: Path):
        """Initialize validator with lesson file path."""
        self.file_path = file_path
        self.content = ""
        self.errors: List[str] = []
        self.warnings: List[str] = []
        
    def validate(self) -> bool:
        """Run all validation checks. Returns True if all checks pass."""
        if not self.file_path.exists():
            self.errors.append(f"File not found: {self.file_path}")
            return False
            
        self.content = self.file_path.read_text(encoding='utf-8')
        
        self._validate_frontmatter()
        self._validate_word_count()
        self._validate_required_sections()
        self._validate_code_blocks()
        self._validate_admonitions()
        self._validate_next_link()
        
        return len(self.errors) == 0
    
    def _validate_frontmatter(self):
        """Check frontmatter for required fields and format."""
        frontmatter_match = re.match(r'^---\n(.*?)\n---', self.content, re.DOTALL)
        if not frontmatter_match:
            self.errors.append("Missing frontmatter block (---...---)")
            return
        
        frontmatter = frontmatter_match.group(1)
        
        # Check for description
        desc_match = re.search(r'description:\s*["\'](.+?)["\']', frontmatter)
        if not desc_match:
            self.errors.append("Missing 'description' field in frontmatter")
        else:
            description = desc_match.group(1)
            if len(description) > self.MAX_DESCRIPTION_LENGTH:
                self.errors.append(
                    f"Description too long: {len(description)} chars "
                    f"(max {self.MAX_DESCRIPTION_LENGTH})"
                )
    
    def _validate_word_count(self):
        """Check if word count is within acceptable range."""
        # Remove frontmatter and code blocks for accurate word count
        content_no_frontmatter = re.sub(r'^---\n.*?\n---\n', '', self.content, flags=re.DOTALL)
        content_no_code = re.sub(r'```.*?```', '', content_no_frontmatter, flags=re.DOTALL)
        
        # Count words
        words = content_no_code.split()
        word_count = len(words)
        
        if word_count < self.MIN_WORDS:
            self.errors.append(
                f"Word count too low: {word_count} words "
                f"(minimum {self.MIN_WORDS})"
            )
        elif word_count > self.MAX_WORDS:
            self.warnings.append(
                f"Word count high: {word_count} words "
                f"(maximum {self.MAX_WORDS} recommended)"
            )
    
    def _validate_required_sections(self):
        """Check that all required sections are present."""
        missing_sections = []
        
        for section in self.REQUIRED_SECTIONS:
            # Look for heading with this text
            pattern = rf'^##+ {re.escape(section)}'
            if not re.search(pattern, self.content, re.MULTILINE):
                missing_sections.append(section)
        
        if missing_sections:
            self.errors.append(
                f"Missing required sections: {', '.join(missing_sections)}"
            )
    
    def _validate_code_blocks(self):
        """Check for minimum number of code examples."""
        code_blocks = re.findall(r'```', self.content)
        code_block_count = len(code_blocks) // 2  # Each block has opening and closing
        
        if code_block_count < self.MIN_CODE_BLOCKS:
            self.warnings.append(
                f"Low code block count: {code_block_count} "
                f"(recommended minimum {self.MIN_CODE_BLOCKS} for technical lessons)"
            )
    
    def _validate_admonitions(self):
        """Check for minimum number of admonitions (notes, tips, warnings)."""
        admonitions = re.findall(r'^:::(note|tip|warning|caution|info)', self.content, re.MULTILINE)
        admonition_count = len(admonitions)
        
        if admonition_count < self.MIN_ADMONITIONS:
            self.warnings.append(
                f"Low admonition count: {admonition_count} "
                f"(recommended minimum {self.MIN_ADMONITIONS} for engagement)"
            )
    
    def _validate_next_link(self):
        """Check for 'What's Next' navigation link."""
        if "What's Next" in self.content:
            # Look for a link in the What's Next section
            whats_next_section = re.search(
                r'##+ What\'s Next.*?(?=^##|\Z)',
                self.content,
                re.MULTILINE | re.DOTALL
            )
            if whats_next_section:
                section_content = whats_next_section.group(0)
                if not re.search(r'\[.+?\]\(.+?\)', section_content):
                    self.warnings.append(
                        "No navigation link found in 'What's Next' section"
                    )
    
    def print_report(self):
        """Print validation report to stdout."""
        print(f"\n{'='*70}")
        print(f"Lesson Validation Report: {self.file_path.name}")
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
        description="Validate lesson file against quality standards"
    )
    parser.add_argument(
        "file_path",
        type=Path,
        help="Path to the lesson Markdown file"
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Treat warnings as errors"
    )
    
    args = parser.parse_args()
    
    validator = LessonValidator(args.file_path)
    is_valid = validator.validate()
    validator.print_report()
    
    # Exit with error code if validation failed
    if not is_valid or (args.strict and validator.warnings):
        sys.exit(1)
    
    sys.exit(0)


if __name__ == "__main__":
    main()
