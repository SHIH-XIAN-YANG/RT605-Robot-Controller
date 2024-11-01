#ifndef _BRT_WIN32_SHARED_MEMORY_H__
#define _BRT_WIN32_SHARED_MEMORY_H__

#include<Windows.h>
#include<Winbase.h>


#include<stdio.h>

namespace brt{
	HANDLE CreateSharedMemory(DWORD dwMaximumSizeLow, LPCSTR lpName, VOID** location);
	/*
	  dwDesiredAccess:(MSDN: https://learn.microsoft.com/en-us/windows/win32/memory/file-mapping-security-and-access-rights)
		 a. FILE_MAP_ALL_ACCESS
		 b. FILE_MAP_EXECUTE
		 c. FILE_MAP_READ
		 d. FILE_MAP_WRITE
	*/
	HANDLE OpenSharedMemory(DWORD  dwDesiredAccess, DWORD dwMaximumSizeLow, LPCSTR lpName, VOID** location);
};
#endif
