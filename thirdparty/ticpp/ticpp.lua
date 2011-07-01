--*****************************************************************************
--*	Author:		RJP Computing <rjpcomputing@gmail.com>
--*	Date:		01/21/2008
--*	Version:	1.02
--* Copyright (C) 2008 RJP Computing
--*
--*	Permission is hereby granted, free of charge, to any person obtaining a copy of
--*	this software and associated documentation files (the "Software"), to deal in
--*	the Software without restriction, including without limitation the rights to
--* use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
--*	the Software, and to permit persons to whom the Software is furnished to do so,
--*	subject to the following conditions:
--*
--* The above copyright notice and this permission notice shall be included in all
--*	copies or substantial portions of the Software.
--*
--*	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
--* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
--*	FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
--*	COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
--*	IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
--* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
--*
--*	NOTES:
--*		- use the '/' slash for all paths.
--*****************************************************************************

--******* Initial Setup ************
--*	Most of the setting are set here.
--**********************************

-- Set the name of your package.
package.name								= "TiCPP"
-- Set this if you want a different name for your target than the package's name.
local targetName							= "ticpp"
-- Set the kind of package you want to create.
if ( options["ticpp-shared"] ) then
	package.kind							= "dll"
else
	package.kind							= "lib"
end
-- Set the files to include/exclude.
package.files								= { matchfiles( "*.cpp", "*.h" ) }
package.excludes							= { "xmltest.cpp" }
-- Setup the output directory options.
--		Note: Use 'libdir' for "lib" kind only.
package.bindir								= "../lib"
package.libdir								= "../lib"
-- Set the defines.
package.defines								= { "TIXML_USE_TICPP" }

--------------------------- DO NOT EDIT BELOW ----------------------------------

--******* GENAERAL SETUP **********
--*	Settings that are not dependant
--*	on the operating system.
--*********************************
-- Package options
addoption( "ticpp-shared", "Build the library as a dll" )

-- Common setup
package.language							= "c++"

-- Set object output directory.
if ( string.find( target or "", ".*-gcc" ) or target == "gnu" ) then
	package.objdir							= ".obj"
end

-- Set the default targetName if none is specified.
if ( string.len( targetName ) == 0 ) then
	targetName								= package.name
end

-- Set the targets.
package.config["Release"].target			= targetName
package.config["Debug"].target				= targetName.."d"

-- Set the build options.
if ( options["dynamic-runtime"] ) then
	package.buildflags						= { "extra-warnings" }
	package.config["Release"].buildflags	= { "no-symbols", "optimize-speed" }
else
	package.buildflags						= { "static-runtime", "extra-warnings" }
	package.config["Release"].buildflags	= { "no-symbols", "optimize-speed" }
end
if ( options["unicode"] ) then
	table.insert( package.buildflags, "unicode" )
end
if ( string.find( target or "", ".*-gcc" ) or target == "gnu" ) then
	table.insert( package.config["Debug"].buildoptions, "-O0" )
end

-- Set the defines.
if ( options["unicode"] ) then
	table.insert( package.defines, { "UNICODE", "_UNICODE" } )
end
table.insert( package.config["Debug"].defines, { "DEBUG", "_DEBUG" } )
table.insert( package.config["Release"].defines, "NDEBUG" )
if ( ( target == "vs2005" ) or ( target == "vs2008" ) ) then
		-- Windows and Visual C++ 2005/2008
	table.insert( package.defines, "_CRT_SECURE_NO_DEPRECATE" )
end

if ( OS == "windows" ) then
--******* WINDOWS SETUP ***********
--*	Settings that are Windows specific.
--*********************************
	-- Set the Windows defines.
	table.insert( package.defines, { "WIN32", "_WINDOWS" } )
else
--******* LINUX SETUP *************
--*	Settings that are Linux specific.
--*********************************
	-- Ignore resource files in Linux.
	table.insert( package.excludes, matchrecursive( "*.rc" ) )
	table.insert( package.buildoptions, "-fPIC" )
end

