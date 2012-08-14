--*****************************************************************************
--*	Author:		RJP Computing <rjpcomputing@gmail.com>
--*	Date:		01/21/2008
--*	Version:	1.02
--* Copyright (C) 2009 RJP Computing
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


--
-- Package options
--
newoption
{
	trigger = "ticpp-shared",
	description = "Build TinyXML++ as a dll"
}

ticpp = {}

function ticpp.GetCustomValue( item )
	local prj = project()
	for _, block in pairs( prj.blocks ) do
		if block[item] then
			return block[item]
		end
	end
	return nil
end

-- Set the name of your package.
project "TiCPP"

	-- Set the files to include/exclude.
	files						{ "*.cpp", "*.h" }
	excludes					{ "xmltest.cpp" }

	-- Set the defines.
	defines						{ "TIXML_USE_TICPP" }

	-- Common setup
	language					"C++"
	flags						{ "ExtraWarnings" }

	--
	-- TinyXML++ dll
	--
	if _OPTIONS["ticpp-shared"] then
		kind 					"SharedLib"
	else
		kind 					"StaticLib"
		if not ticpp.GetCustomValue( "targetdir" ) then
			targetdir( solution().basedir .. "/lib" )
		end
	end

	--
	-- Dynamic Runtime
	--
	if not _OPTIONS["dynamic-runtime"] then
		flags					{ "StaticRuntime" }
	end

	--
	-- Operating Systems specific
	--
	if os.is( "windows" ) then
		defines					{ "WIN32", "_WINDOWS" }
	else
		excludes				{ "**.rc" }		-- Ignore resource files in Linux.
		buildoptions			{ "-fPIC" }
	end

	--
	-- Unicode
	--
	configuration "unicode"
		flags					{ "Unicode" }
		defines					{ "UNICODE", "_UNICODE" }

	--
	-- GCC compilers
	--
	configuration { "code*", "gmake" }
		objdir					".obj"
		buildoptions			{ "-O0" }

	--
	-- Visual Studio
	--
	configuration "vs*"
		-- Windows and Visual C++ 2005/2008
		defines					{ "_CRT_SECURE_NO_DEPRECATE" }

	configuration( "vs2008 or vs2010" )
		-- multi-process building
		flags( "NoMinimalRebuild" )
		buildoptions( "/MP" )

	--
	-- Release/Debug
	--

	-- Set the targets.
	configuration "Release"
		targetname				( "ticpp" )
		defines					{ "NDEBUG" }
		flags					{ "OptimizeSpeed" }
	configuration "Debug"
		targetname 				( "ticppd" )
		defines					{ "DEBUG", "_DEBUG" }
		flags					{ "Symbols" }
