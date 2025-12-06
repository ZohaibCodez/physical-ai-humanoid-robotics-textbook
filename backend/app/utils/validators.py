"""
Email and password validation utilities.
"""

import re
from typing import Tuple


def validate_email(email: str) -> Tuple[bool, str]:
    """
    Validate email format and domain structure.
    
    Args:
        email: Email address to validate
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    # Basic email regex (RFC 5322 simplified)
    email_pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    
    if not email:
        return False, "Email is required"
    
    if not re.match(email_pattern, email):
        return False, "Invalid email format"
    
    # Check for common typos
    if '..' in email:
        return False, "Email contains consecutive dots"
    
    if email.startswith('.') or email.endswith('.'):
        return False, "Email cannot start or end with a dot"
    
    # Split and validate parts
    try:
        local, domain = email.rsplit('@', 1)
        
        if not local:
            return False, "Email local part is empty"
        
        if not domain:
            return False, "Email domain is empty"
        
        if '.' not in domain:
            return False, "Email domain must contain at least one dot"
        
    except ValueError:
        return False, "Invalid email structure"
    
    return True, ""


def validate_password(password: str) -> Tuple[bool, str]:
    """
    Validate password meets security requirements.
    
    Requirements:
    - Minimum 8 characters
    - Maximum 72 bytes (bcrypt limit)
    - (Optional complexity requirements can be added here)
    
    Args:
        password: Password to validate
        
    Returns:
        Tuple of (is_valid, error_message)
    """
    if not password:
        return False, "Password is required"
    
    if len(password) < 8:
        return False, "Password must be at least 8 characters"
    
    # Check bcrypt 72-byte limit
    if len(password.encode('utf-8')) > 72:
        return False, "Password is too long (maximum 72 bytes)"
    
    # Optional: Add complexity requirements for production
    # has_uppercase = any(c.isupper() for c in password)
    # has_lowercase = any(c.islower() for c in password)
    # has_digit = any(c.isdigit() for c in password)
    # has_special = any(c in '!@#$%^&*()_+-=[]{}|;:,.<>?' for c in password)
    # 
    # if not (has_uppercase and has_lowercase and has_digit):
    #     return False, "Password must contain uppercase, lowercase, and digit"
    
    return True, ""
