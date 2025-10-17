#!/usr/bin/env python3
import re
import os

# 配置参数
FILE_EXTENSIONS = ['.cpp', '.h', '.hpp', '.cc', '.cmake', 'CMakeLists.txt']  # 需要处理的文件扩展名
OLD_YEAR = 2024   # 原年份（或范围的结束年份）
NEW_YEAR = 2025   # 新年份

# 匹配MIT版权声明的正则模式
PATTERN = re.compile(
    r'(Copyright\s+\(c\)\s+)(\d{4})(?:\s*-\s*(\d{4}))?', 
    re.IGNORECASE
)

def update_year_in_file(filepath):
    with open(filepath, 'r+') as f:
        content = f.read()
        new_content = PATTERN.sub(repl, content)
        if content != new_content:
            f.seek(0)
            f.write(new_content)
            f.truncate()
            print(f"Updated: {filepath}")

def repl(match):
    prefix = match.group(1)  # "Copyright (c) "
    start_year = match.group(2)  # 起始年份（或单年份）
    end_year = match.group(3)   # 结束年份（如果有范围）
    
    # 如果是年份范围（如2020-2023）
    if end_year and int(end_year) == OLD_YEAR:
        return f"{prefix}{start_year}-{NEW_YEAR}"
    # 如果是单年份且等于OLD_YEAR
    elif not end_year and int(start_year) == OLD_YEAR:
        return f"{prefix}{NEW_YEAR}"
    # 其他情况不替换（避免误修改）
    else:
        return match.group(0)

# 遍历所有文件
for root, dirs, files in os.walk('.'):
    for file in files:
        if any(file.endswith(ext) for ext in FILE_EXTENSIONS):
            update_year_in_file(os.path.join(root, file))