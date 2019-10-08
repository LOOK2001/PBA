#ifndef __PBA_ASCIIPARSER_H__
#define __PBA_ASCIIPARSER_H__

#include <string>
#include <vector>
using namespace std;

namespace pba
{

	class AsciiParser
	{
	public:

		AsciiParser();
		~AsciiParser() {}


		const bool ParseFile(const string& filename);

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

#endif