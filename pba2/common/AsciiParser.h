#ifndef __PBA_ASCIIPARSER_H__
#define __PBA_ASCIIPARSER_H__

#include <string>
#include <vector>
#include "Vector.h"
using namespace std;

namespace pba
{

	struct meshData 
	{
		std::vector<Vector> Positions;
		//std::vector<float[2]> TCoords;
		std::vector<Vector> Normals;
		std::vector<unsigned int> Indices;
	};

	class AsciiParser
	{
	public:

		AsciiParser();
		~AsciiParser() {}


		const bool ParseFile(const string& filename, meshData& mesh);

		void Rewind();

		const bool GetToken();

		// make sure we are pointing to a valid token.
		const bool IsToken() const;

		const bool IsText()    const;
		const bool IsInteger() const;
		const bool IsFloat()   const;
		const bool IsSeparator()   const;
		const bool IsOther()   const;
		const bool IsEOL()   const;

		const int IntegerValue()  const;
		const double FloatValue() const;
		const string& TextValue() const;

		void AddWhiteSpaceCharacter(const string& ws);

		const vector<string>& Tokens() const { return tokens; }

	private:

		enum TokenType { OTHER = 0, STRING, INTEGER, FLOAT, SEPARATOR, EOL };
		vector<string>    tokens;
		vector<TokenType> token_type;
		int current_token;
		const string blank;
		string whitespace_characters;
		const string valid_integer_characters;
		const string valid_float_characters;
		const string valid_separator_characters;

		// turn a line of input text into tokens
		void Tokenize(const string& line);
		void AddToken(const string& line);
		void AddEOLToken();
		void AddOtherToken();
	};
}

namespace algorithm
{
	/// Split a String into a string array at a given token
	inline void split(const std::string& in,
		std::vector<std::string>& out,
		std::string token)
	{
		out.clear();

		std::string temp;

		for (int i = 0; i < int(in.size()); i++)
		{
			std::string test = in.substr(i, token.size());

			if (test == token)
			{
				if (!temp.empty())
				{
					out.push_back(temp);
					temp.clear();
					i += (int)token.size() - 1;
				}
				else
				{
					//out.push_back("");
				}
			}
			else if (i + token.size() >= in.size())
			{
				temp += in.substr(i, token.size());
				out.push_back(temp);
				break;
			}
			else
			{
				temp += in[i];
			}
		}
	}

	// Get tail of string after first token and possibly following spaces
	inline std::string tail(const std::string& in)
	{
		size_t token_start = in.find_first_not_of(" \t");
		size_t space_start = in.find_first_of(" \t", token_start);
		size_t tail_start = in.find_first_not_of(" \t", space_start);
		size_t tail_end = in.find_last_not_of(" \t");
		if (tail_start != std::string::npos && tail_end != std::string::npos)
		{
			return in.substr(tail_start, tail_end - tail_start + 1);
		}
		else if (tail_start != std::string::npos)
		{
			return in.substr(tail_start);
		}
		return "";
	}

	// Get first token of string
	inline std::string firstToken(const std::string& in)
	{
		if (!in.empty())
		{
			size_t token_start = in.find_first_not_of(" \t");
			size_t token_end = in.find_first_of(" \t", token_start);
			if (token_start != std::string::npos && token_end != std::string::npos)
			{
				return in.substr(token_start, token_end - token_start);
			}
			else if (token_start != std::string::npos)
			{
				return in.substr(token_start);
			}
		}
		return "";
	}
}

#endif