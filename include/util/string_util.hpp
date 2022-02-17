#ifndef STRING_UTIL_HPP
#define STRING_UTIL_HPP

#include <cctype>

#include <random>
#include <string>
#include <vector>
#include <algorithm>

#include <random>
#include <sstream>

namespace osrm
{
namespace util
{

// precision:  position after decimal point
// length: maximum number of digits including comma and decimals
// work with negative values to prevent overflowing when taking -value
template <int length, int precision> char *printInt(char *buffer, int value)
{
    static_assert(length > 0, "length must be positive");
    static_assert(precision > 0, "precision must be positive");

    const bool minus = [&value] {
        if (value >= 0)
        {
            value = -value;
            return false;
        }
        return true;
    }();

    buffer += length - 1;
    for (int i = 0; i < precision; ++i)
    {
        *buffer = '0' - (value % 10);
        value /= 10;
        --buffer;
    }
    *buffer = '.';
    --buffer;

    for (int i = precision + 1; i < length; ++i)
    {
        *buffer = '0' - (value % 10);
        value /= 10;
        if (value == 0)
        {
            break;
        }
        --buffer;
    }

    if (minus)
    {
        --buffer;
        *buffer = '-';
    }
    return buffer;
}

inline std::string escape_JSON(const std::string &input)
{
    // escape and skip reallocations if possible
    std::string output;
    output.reserve(input.size() + 4); // +4 assumes two backslashes on avg
    for (const char letter : input)
    {
        switch (letter)
        {
        case '\\':
            output += "\\\\";
            break;
        case '"':
            output += "\\\"";
            break;
        case '/':
            output += "\\/";
            break;
        case '\b':
            output += "\\b";
            break;
        case '\f':
            output += "\\f";
            break;
        case '\n':
            output += "\\n";
            break;
        case '\r':
            output += "\\r";
            break;
        case '\t':
            output += "\\t";
            break;
        default:
            output.append(1, letter);
            break;
        }
    }
    return output;
}

inline std::size_t URIDecode(const std::string &input, std::string &output)
{
    auto src_iter = std::begin(input);
    const auto src_end = std::end(input);
    output.resize(input.size() + 1);
    std::size_t decoded_length = 0;
    for (decoded_length = 0; src_iter != src_end; ++decoded_length)
    {
        if (src_iter[0] == '%' && src_iter[1] && src_iter[2] && isxdigit(src_iter[1]) &&
            isxdigit(src_iter[2]))
        {
            std::string::value_type a = src_iter[1];
            std::string::value_type b = src_iter[2];
            a -= src_iter[1] < 58 ? 48 : src_iter[1] < 71 ? 55 : 87;
            b -= src_iter[2] < 58 ? 48 : src_iter[2] < 71 ? 55 : 87;
            output[decoded_length] = 16 * a + b;
            src_iter += 3;
            continue;
        }
        output[decoded_length] = *src_iter++;
    }
    output.resize(decoded_length);
    return decoded_length;
}

inline std::size_t URIDecodeInPlace(std::string &URI) { return URIDecode(URI, URI); }


// trim from start (in place)
static void ltrim(std::string &s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
		return !std::isspace(ch);
	}));
}

// trim from end (in place)
static void rtrim(std::string &s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
		return !std::isspace(ch);
	}).base(), s.end());
}

// trim from both ends (in place)
static void trim(std::string &s) {
	ltrim(s);
	rtrim(s);
}

// trim from start (copying)
static std::string ltrim_copy(std::string s) {
	ltrim(s);
	return s;
}

// trim from end (copying)
static std::string rtrim_copy(std::string s) {
	rtrim(s);
	return s;
}

// trim from both ends (copying)
static std::string trim_copy(std::string s) {
	trim(s);
	return s;
}

static std::string iso_8859_1_to_utf8(std::string &str)
{
	std::string strOut;
	for (std::string::iterator it = str.begin(); it != str.end(); ++it)
	{
		uint8_t ch = *it;
		if (ch < 0x80) {
			strOut.push_back(ch);
		}
		else {
			strOut.push_back(0xc0 | ch >> 6);
			strOut.push_back(0x80 | (ch & 0x3f));
		}
	}
	return strOut;
}



namespace uuid {
static std::random_device              rd;
static std::mt19937                    gen(rd());
static std::uniform_int_distribution<> dis(0, 15);
static std::uniform_int_distribution<> dis2(8, 11);

static std::string generate_uuid_v4() {
	std::stringstream ss;
	int i;
	ss << std::hex;
	for (i = 0; i < 8; i++) {
		ss << dis(gen);
	}
	ss << "-";
	for (i = 0; i < 4; i++) {
		ss << dis(gen);
	}
	ss << "-4";
	for (i = 0; i < 3; i++) {
		ss << dis(gen);
	}
	ss << "-";
	ss << dis2(gen);
	for (i = 0; i < 3; i++) {
		ss << dis(gen);
	}
	ss << "-";
	for (i = 0; i < 12; i++) {
		ss << dis(gen);
	};
	return ss.str();
}
} // namespace uuid
} // namespace util
} // namespace osrm

#endif // STRING_UTIL_HPP
