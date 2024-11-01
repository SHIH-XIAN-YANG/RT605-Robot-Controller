/* Circular Buffer
  - Design: B. R. Tseng
  - Data: 2021/03/30
 */
 // The last edition: 2021/05/11, B. R. Tseng
 // ================================================================================
 #ifndef __CIRCULAR_BUFFER_H__
 #define __CIRCULAR_BUFFER_H__
 #include<vector>
 #include<array>
 #include<utility>
 // ================================================================================
namespace brt {
#pragma region  Self define element type.  (Not finished yet.)
	template <class T, unsigned int len>
	class CBuffer {
	private:
		std::array<T, len> _pBuffer;
		unsigned int _Length;  // Buffer capacity.
		size_t _Depth;         // The dimension of element or element depth. (unit: byte)
		bool _WD_overflow;
		class std::array<T, len>::iterator _itBegin, _itEnd;
	public:
		class std::array<T, len>::iterator itWirte, itRead;
		CBuffer(void) {
			_WD_overflow = 0;
			_Length = len;
			_Depth = sizeof(T);
			_itBegin = _pBuffer.begin();
			_itEnd = _pBuffer.end();
			itWirte = _pBuffer.begin();;
			itRead = _pBuffer.begin();;
		}
		void CB_Write(T* data) {
			*itWirte = *data;
			if (++itWirte == _itEnd) {
				itWirte = _itBegin;
				_WD_overflow = 1;
			}
		}
		void CB_Read(T* readData) {
			if (ReadPermitted) {
				*readData = *itRead;
				if (++itRead == _itEnd)
					itRead = _itBegin;
			}
		}
		bool ReadPermitted(void) {
			if (itWirte == itRead)
				return false;
			else
				return true;
		};
#pragma endregion 
		//
#pragma region using STL container: std::array  (Not finished yet.)
		template <class TYPE, unsigned int depth, unsigned int len>
		class CBufferArray { // Under STL container: std::array
		private:
			unsigned int _Depth;
			unsigned int _Length;
			std::array<std::array<TYPE, depth>, len>* _pBuffer;
			class std::array<std::array<TYPE, depth>, len>::iterator  _itBegin,
				_itEnd;
			bool _WD_overflow;
		public:

			class std::array<std::array<TYPE, depth>, len> itRead, itWrite;
			CBufferArray(void) {
				_WD_overflow = 0;
				_Depth = depth;
				_Length = len;
				_pBuffer = new std::array<std::array<TYPE, depth>, len>; // construct the buffer
				_itBegin = _pBuffer->begin();
				_itEnd = _pBuffer->end();
				itRead = _pBuffer->begin();   // assign read pointer
				itWrite = _pBuffer->begin();  // assing write pointer
			}
			~CBufferArray(void) {
				delete _pBuffer;
			}
		};
#pragma endregion
		//
#pragma region using STL container: std::vector  (Not finished yet.)
		template <class TYPE, unsigned int depth, unsigned int len>
		class CBufferVector {
		private:
			unsigned int _Depth; // Buffer depth or vector dimension
			unsigned int _Length; // Data length
			std::vector<std::vector<TYPE>>* _pBuffer; // Memory Buffer Register
			/* -(Note: The structure of buffer layer):
				Buffer[data serial index][data vector index]
				-----------------------------------------------------------
				example:

					double time[3] = {0.0, 0.01, 0.02}, q1[3] = {0, 20, 30}, q2[3] = {90, 45, 45}, q3[3] = {0, 0, 0};

					=> build a buffer: std::vector<std::vector<double>> buf1 = { {time[0], q1[0], q2[0], q3[0]},
																				 {time[1], q1[1], q2[1], q3[1]},
																				 {time[2], q1[2], q2[2], q3[2]} };
					-The dimension/depth of buffer is "4." (the dimension of data vector)

					-The length of buffer is "3." (the amount of data serial)
			*/
			bool _WD_overflow;
			class std::vector<std::vector<TYPE>>::iterator  _itBegin,   // the Iterator pointing to the first element
				_itEnd;     // the Iterator referring to the past-the-end element in the buffer
/* example: itBegin and itEnd forming a semi-open interval  [ itBegin, itEnd )
	  ____ ____ ____ ____ ____ ____ ____
	 |    |    |    |    |    |    |    |
	 |____|____|____|____|____|____|____|
	   /\                                 /\
	_itBegin                            _itEnd

*/
		public:
			// ---------------- Member Variables: ----------------
			class std::vector<std::vector<TYPE>>::iterator  itRead,    // the Iterator of Read operation
				itWrite;   // the Iterator of Write operation

// ---------------- Member Function: ----------------
	// -I/P: 
	//		- depth<U64>: define the depth of buffer
	//		- len<U64>: define the length of data 
			CBufferVector(void) {
				_WD_overflow = 0;
				_Depth = depth;
				_Length = len;
				_pBuffer = new std::vector<std::vector<TYPE>>(len, std::vector<TYPE>(depth)); // construct the buffer
				_itBegin = _pBuffer->begin();
				_itEnd = _pBuffer->end();
				itRead = _pBuffer->begin();   // assign read pointer
				itWrite = _pBuffer->begin();  // assing write pointer
			}
			~CBufferVector(void) {
				delete _pBuffer;
			}
			void append(const std::vector<TYPE>& data) {
				*(itWrite++) = data; // the data vector of the present index
				if (itWrite == _pBuffer->end()) { // checking for the overflow condiction
					itWrite = _pBuffer->begin();
					_WD_overflow = 1;
				}
			}
			bool pop(class std::vector<std::vector<TYPE>>::iterator& itConsumer) {
				bool nRet{ false };
				if (_WD_overflow == 0 && itRead < itWrite) {
					*(itConsumer++) = *itRead;
					if (itRead == _pBuffer->end()) {
						itRead = _pBuffer->begin();
					}
					nRet = true;
				}
				else if (_WD_overflow == 1 && itRead > itWrite) {
					*(itConsumer++) = *itRead;
					if (itRead == _pBuffer->end()) {
						itRead = _pBuffer->begin();
						_WD_overflow = 0;
					}
					nRet = true;
				}
				else {
					nRet = false;
				}
				return nRet;
			}

		};
#pragma endregion
		//
#pragma region 
#pragma endregion
	};
}
#endif
 /* =================== Examples: ==================================
	CBufferN<double> cb1(3, 100);
	std::vector<std::vector<double>> data = { {1, 1, 1}, {2, 2, 2}, {3, 3, 3} };
	for(auto& val:data)
		cb1.append(val);
	cb1.append(data.at(0));

 */
