import re

# Read the README.md file
with open('README.md', 'r', encoding='utf-8') as f:
    content = f.read()

# Remove emojis using regex pattern for Unicode emoji ranges
emoji_pattern = re.compile(
    "["
    "\U0001F600-\U0001F64F"  # emoticons
    "\U0001F300-\U0001F5FF"  # symbols & pictographs
    "\U0001F680-\U0001F6FF"  # transport & map symbols
    "\U0001F1E0-\U0001F1FF"  # flags (iOS)
    "\U00002702-\U000027B0"
    "\U000024C2-\U0001F251"
    "]+", flags=re.UNICODE
)

cleaned_content = emoji_pattern.sub('', content)

# Write the cleaned content back to README.md
with open('README.md', 'w', encoding='utf-8') as f:
    f.write(cleaned_content)

print("Emojis removed from README.md")