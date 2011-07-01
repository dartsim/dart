#if !defined ( __TINY_XML_MOCK__ )
#define __TINY_XML_MOCK__

namespace ticpp
{
	namespace mock
	{
		struct Node;
		struct Element;
		struct Declaration;

		struct NodeBase
		{
			NodeBase( void )
			{
			}
			MOCK_METHOD0( Clear, void ( void ) );
			MOCK_METHOD1( SetValue, void( const std::string& ) );
			MOCK_METHOD1( InsertEndChild, void( const Node& ) );
			MOCK_METHOD1( LinkEndChild, void( const Node* ) );
			MOCK_METHOD1( SetText, void ( const double ) );
			MOCK_METHOD1( SetText, void ( const std::string& ) );
			MOCK_METHOD1( GetValue, void ( std::string* ) );
			MOCK_METHOD1( SetIntText, void ( const unsigned int ) );
			MOCK_METHOD1( RemoveChild, void ( Node* ) );
			MOCK_CONST_METHOD0( Clone, const Node& ( void ) );		//This is done because auto_ptr's copy constructor is declared explicit and generates a compiler error inside googlemock.
			MOCK_CONST_METHOD0( ToElement, Element*( void ) );
			MOCK_CONST_METHOD0( NoChildren, bool ( void ) );
			MOCK_CONST_METHOD0( FirstChild, Node*( void ) );
			MOCK_CONST_METHOD1( FirstChild, Node*( const std::string& ) );
			MOCK_CONST_METHOD2( FirstChild, Node*( const std::string&, bool ) );
			MOCK_CONST_METHOD1( IterateChildren, Node*( Node* ) );
			MOCK_CONST_METHOD2( IterateChildren, Node*( const std::string, Node* ) );
			MOCK_CONST_METHOD0( FirstChildElement, Element*() );
			MOCK_CONST_METHOD1( FirstChildElement, Element*( bool ) );
			MOCK_CONST_METHOD0( Value, std::string( void ) );
			virtual ~NodeBase( void )
			{
			}
		};

		struct Node : public StrictMock< NodeBase >
		{
			//This is the work around for the Clone method not returning auto_ptr.
			const Node* release( void ) const
			{
				return this;
			}

			Node( void ) : StrictMock< NodeBase >()
			{
			}

			Node( const Node& ) : StrictMock< NodeBase >()
			{
			}

			bool operator==( const Node& other ) const
			{
				return *this == other;
			}
			virtual ~Node( void )
			{
			}
		};

		struct ElementBase : public Node
		{
			ElementBase( const std::string& ) : Node()
			{
			}
			MOCK_CONST_METHOD1( GetAttribute, std::string( const std::string& ) );
			MOCK_CONST_METHOD2( GetAttribute, std::string( std::string, bool ) );
			MOCK_METHOD2( SetAttribute, void ( const std::string&, const int ) );
			MOCK_METHOD2( SetAttribute, void ( const std::string&, const std::string& ) );
			MOCK_CONST_METHOD0( GetText, std::string( void ) );
			MOCK_CONST_METHOD1( GetText, std::string( bool ) );
			virtual ~ElementBase( void )
			{
			}
		};

		struct Element : public StrictMock< ElementBase >
		{
			explicit Element( const std::string& name = std::string() ) : StrictMock< ElementBase >( name )
			{
			}

			bool operator==( const Element& other ) const
			{
				return *this == other;
			}
			virtual ~Element( void )
			{
			}
		};

		struct DocumentBase : public Node
		{
			DocumentBase( void ) : Node()
			{
			}
			DocumentBase( const std::string& ) : Node()
			{
			}
			MOCK_METHOD0( SaveFile, void ( void ) );
			MOCK_METHOD1( SaveFile, void ( const std::string& ) );
			MOCK_METHOD0( LoadFile, void ( void ) );
			MOCK_METHOD1( LoadFile, void ( const std::string& ) );
			MOCK_METHOD1( Parse, void ( const std::string& ) );

			virtual ~DocumentBase( void )
			{
			}
		};

		struct Document : public StrictMock< DocumentBase >
		{
			Document( void ) : StrictMock< DocumentBase >()
			{
			}
			Document( const std::string& name ) : StrictMock< DocumentBase >( name )
			{
			}
			MOCK_CONST_METHOD0( GetDocument, Document*( void ) );
			virtual ~Document( void )
			{
			}
		};

		struct DeclarationBase : public Node
		{
			DeclarationBase( void ) : Node()
			{
			}

			DeclarationBase( const std::string&, const std::string&, const std::string& ) : Node()
			{
			}
			virtual ~DeclarationBase( void )
			{
			}
		};

		struct Declaration : public StrictMock< DeclarationBase >
		{
			Declaration( void ) : StrictMock< DeclarationBase >()
			{
			}

			Declaration( const std::string& p1, const std::string& p2, const std::string& p3 ) : StrictMock< DeclarationBase >( p1, p2, p3 )
			{
			}
			virtual ~Declaration( void )
			{
			}
		};

		struct Comment : public Node
		{
			Comment( void )
			{
			}
			virtual ~Comment( void )
			{
			}
		};

		inline std::ostream& operator << ( std::ostream& os, const Node& )
		{
			return( os );
		}
	}
}
#endif // !defined ( __TINY_XML_MOCK__ )
