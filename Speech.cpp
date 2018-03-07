#include <sapi.h>
#include <iostream>
#include <string>

using namespace std;

class Speech{
public:
	ISpVoice * pVoice = NULL;


	Speech()
	{
		
		HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
		if (FAILED(hr))
		{
			cout << "Text to speech failed!" << endl;
		}
	}

	void Talk(string text)
	{
		std::wstring stemp = std::wstring(text.begin(), text.end());
		LPCWSTR sw = stemp.c_str();
		pVoice->Speak(sw, 0, NULL);
		cout << "should have said something" << endl;
	}

	~Speech()
	{
		pVoice->Release();
		pVoice = NULL;
		
	}
};