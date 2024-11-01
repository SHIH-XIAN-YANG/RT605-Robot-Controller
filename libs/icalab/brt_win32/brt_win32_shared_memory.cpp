#include"brt_win32/brt_win32_shared_memory.h"


HANDLE brt::CreateSharedMemory(DWORD dwMaximumSizeLow, LPCSTR lpName, VOID** location){
	HANDLE shm_handle = CreateFileMapping(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, dwMaximumSizeLow, (LPCWSTR)lpName);
	*location = MapViewOfFile(shm_handle, FILE_MAP_ALL_ACCESS, 0, 0, dwMaximumSizeLow);
   if (*location == NULL)
   {
      printf( "Could not map view of file, error-code:%d.\n", GetLastError());
      CloseHandle(shm_handle);
   }
	return shm_handle;
}

HANDLE brt::OpenSharedMemory(DWORD  dwDesiredAccess, DWORD dwMaximumSizeLow, LPCSTR lpName, VOID** location){
	HANDLE  hMapFile = OpenFileMapping(dwDesiredAccess,   // read/write access
										FALSE,            // do not inherit the name
									 (LPCWSTR)lpName);          // name of mapping object
	*location = MapViewOfFile(hMapFile, dwDesiredAccess, 0, 0, dwMaximumSizeLow);
	
	return hMapFile;
}



/* Example: 
// ============================================
// ------------- Process A: a.cpp -------------
// ============================================

#include<iostream>

#include"brt_win32_shared_memory.h"

int main(int argc, char* argv[]){
	std::cout << "Entering the process A." << std::endl;
	std::cout << "ProcessA: creating the shared memory ......." << std::endl;
	
	pBuffer pbuf1 = NULL;
	HANDLE shm_handle = CreateSharedMemory(sizeof(Buffer), "Shared memory", (void**)&pbuf1);
	


	
	
	STARTUPINFO si;
    PROCESS_INFORMATION pi;
	
	ZeroMemory(&si, sizeof(si));
	char title[] = "Process B";
	si.lpTitle = title;
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));
	
	if(! CreateProcess("./b.exe", 
								NULL, 
								NULL, 
								NULL, 
								false, 
								0,
								NULL,
								NULL,
								&si,
								&pi)){
		printf( "CreateProcess failed (%d).\n", GetLastError() );
		return -1;
    }
	else{
		std::cout << "ProcessA: Open the process B." << std::endl;
	}
	
	std::cout << "Show data[0]:" << std::endl;
	for(int i = 0; i<1000; ++i){
		std::cout << pbuf1->data[0] << std::endl;
		Sleep(500);
	}
	std::cout << "ProcessA: Terminating the process A." << std::endl;
	system("pause");
	return 0;
}
// ============================================
// ------------- Process B: b.cpp -------------
// ============================================

#include<iostream>

#include"brt_win32_shared_memory.h"

typedef struct Buffer{
	HANDLE m_handle;
	int data[128];
}*pBuffer, Buffer;

int main(int argc, char* argv[]){
	std::cout << "Entering the process B." << std::endl;
	
	Buffer* pbuf1;
	
	
	
	HANDLE  hMapFile = OpenSharedMemory(FILE_MAP_ALL_ACCESS, sizeof(Buffer), "Shared memory" , &pbuf1);
	
	pbuf1->data[0] = 99;
	
	Sleep(500000);
	
	std::cout << "ProcessB: Terminating the process B." << std::endl;
	
	system("pause");
	return 0;
}
*/