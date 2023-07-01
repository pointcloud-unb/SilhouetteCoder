#include <fstream>
#include <string>
#include <vector>

static bool
compareSeparators(char aChar, const char* const sep)
{
  int i = 0;
  while (sep[i] != '\0') {
    if (aChar == sep[i])
      return false;
    i++;
  }
  return true;
}

static bool
getTokens(
  const char* str, const char* const sep, std::vector<std::string>& tokens)
{
  // Clear the vector first
  if (!tokens.empty())
    tokens.clear();
  // Start the string with nothing
  std::string buf = "";
  size_t i = 0;
  size_t length = ::strlen(str);
  while (i < length) {
    // Compare the character with the separator to cut the line in the correct place
    // compareSeparators function returns true if the character is not equal to the separator
    if (compareSeparators(str[i], sep)) {
      // If its not the separator, add it to the buf
      buf += str[i];
    } else if (buf.length() > 0) {
      // If the separator, put it in the tokens vector
      tokens.push_back(buf);
      // Resets the buff to get the next word
      buf = "";
    }
    i++;
  }
  // For the last word in the line there is no separator, so if the buf is not empty, add it to the vector
  if (!buf.empty())
    tokens.push_back(buf);
  return !tokens.empty();
}