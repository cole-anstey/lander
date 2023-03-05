// Copyright (C) 2023 Cole Anstey
#pragma once

#include <wincodec.h>

#define TRANSPARENT_COLOR Color::White

Bitmap* _bmpScreen;
HGLOBAL _SCREEN_XPM_hBuffer;

//#ifdef GDI_PLUS
//if (_hBuffer)
//{
//    ::GlobalUnlock(_hBuffer);
//    ::GlobalFree(_hBuffer);
//    _hBuffer = NULL;
//}
//#endif  /*GDI_PL

bool LoadImageFromMemory(void const* pData, DWORD imageSize, Bitmap** ppBitamp, HGLOBAL* phBitmap);
bool LoadImage(LPCTSTR pName, LPCTSTR pType, HMODULE hModule, Bitmap** ppBitamp, HGLOBAL* phBitmap);

void DrawImage2(Graphics* gc, Bitmap* pbitmap, int x, int y, int width, int height);
void StretchImage(Graphics* gc, Bitmap* pbitmap, int x, int y, int width, int height, int scale);

void SetPixel(Graphics* gc, Bitmap* pbitmap, int x, int y);

BOOL InitialiseImages(HMODULE hModule)
{
  if (::CoInitialize(NULL) == S_FALSE)
  {
    return false;
  }

  //  BOOL initialised = false;
  //
  //  int i;
  //
  //  for(i = 0; i < IMAGE_COUNT; i++)
  //  {
  //    WORD image_id = image_ids[i];
  //    pxpmimage xpm = xpms[i];
  //
  //    if (image_id != NULL)
  //    {
  //      // Create a bitmap from an application resource.
  //      if (!LoadImage(MAKEINTRESOURCE(image_id), L"PNG", hModule, &xpm->pixmap, &xpm->_hBuffer))
  //      {
  //        initialised = false;
  //
  //        break;
  //      }
  //    }
  //  }

  ::CoUninitialize();

  return true;
}

#pragma region Resource Helpers
/*******************************************/
/* CGdiPlusBitmapResource::LoadImageFromMemory */
/*****************************************/
bool LoadImageFromMemory(void const* pData, DWORD imageSize, Bitmap** ppBitamp, HGLOBAL* phBitmap)
{
  *phBitmap = ::GlobalAlloc(GMEM_MOVEABLE, imageSize);

  if (*phBitmap) {
    void* pBuffer = ::GlobalLock(*phBitmap);

    if (pBuffer) {
      ::CopyMemory(pBuffer, pData, imageSize);

      IStream* pStream = NULL;

      if (::CreateStreamOnHGlobal(*phBitmap, false, &pStream) == S_OK) {
        *ppBitamp = Gdiplus::Bitmap::FromStream(pStream);

        pStream->Release();

        if (*ppBitamp) {
          if ((*ppBitamp)->GetLastStatus() == Gdiplus::Ok) {
            return true;
          }

          delete* ppBitamp;
          *ppBitamp = NULL;
        }
      }

      ::GlobalUnlock(*phBitmap);
    }

    ::GlobalFree(*phBitmap);
    *phBitmap = NULL;
  }

  return false;
}

bool LoadImage(LPCTSTR pName, LPCTSTR pType, HMODULE hModule, Bitmap** ppBitamp, HGLOBAL* phBitmap)
{
  HRSRC hResource = ::FindResourceW(hModule, pName, pType);

  if (!hResource) {
    return false;
  }

  DWORD imageSize = ::SizeofResource(hModule, hResource);

  if (!imageSize) {
    return false;
  }

  // For 32-bit Windows applications, it is not necessary to free the resources loaded using LoadResource.
  // Unlike LoadBitmap() (and unlike 16 - bit Windows), LoadResource() does not allocate any additional resources in Win32, 
  // but rather "points" to the resource in the memory - mapped image of the already loaded module.
  // Therefore, there is nothing for FreeResource() to clean up in Win32.
  HGLOBAL hGlobal = ::LoadResource(hModule, hResource);

  if (!hGlobal) {
    return false;
  }

  const void* pResourceData = ::LockResource(hGlobal);

  if (!pResourceData) {
    return false;
  }

  return LoadImageFromMemory(pResourceData, imageSize, ppBitamp, phBitmap);
}

bool LoadDataFromMemory(void const* pData, DWORD imageSize, void** ppData, HGLOBAL* phData)
{
  *phData = ::GlobalAlloc(GMEM_MOVEABLE, imageSize);

  if (*phData) {
    *ppData = ::GlobalLock(*phData);

    if (*ppData) {
      ::CopyMemory(*ppData, pData, imageSize);

      return true;
    }

    ::GlobalFree(*phData);
    *phData = NULL;
  }

  return false;
}

bool LoadData(LPCTSTR pName, LPCTSTR pType, HMODULE hModule, void** ppData, HGLOBAL* phData)
{
  HRSRC hResource = ::FindResourceW(hModule, pName, pType);

  if (!hResource) {
    return false;
  }

  DWORD imageSize = ::SizeofResource(hModule, hResource);

  if (!imageSize) {
    return false;
  }

  // For 32-bit Windows applications, it is not necessary to free the resources loaded using LoadResource.
  // Unlike LoadBitmap() (and unlike 16 - bit Windows), LoadResource() does not allocate any additional resources in Win32, 
  // but rather "points" to the resource in the memory - mapped image of the already loaded module.
  // Therefore, there is nothing for FreeResource() to clean up in Win32.
  HGLOBAL hGlobal = ::LoadResource(hModule, hResource);

  if (!hGlobal) {
    return false;
  }

  const void* pResourceData = ::LockResource(hGlobal);

  if (!pResourceData) {
    return false;
  }

  return LoadDataFromMemory(pResourceData, imageSize, ppData, phData);
}
#pragma region

void DrawImage2(Graphics* gc, Bitmap* pbitmap, int x, int y, int width, int height)
{
  StretchImage(gc, pbitmap, x, y, width, height, 1);
}

void StretchImage(Graphics* gc, Bitmap* pbitmap, int x, int y, int width, int height, int scale)
{
  ImageAttributes* attr = new ImageAttributes();
  attr->SetColorKey(TRANSPARENT_COLOR, TRANSPARENT_COLOR);

  Rect destRect(x, y, width * scale, height * scale);
  gc->DrawImage(pbitmap, destRect, 0, 0, pbitmap->GetWidth(), pbitmap->GetHeight(), UnitPixel, attr);

  delete attr;
}

void SetPixel(Graphics* gc, Bitmap* pbitmap, int x, int y)
{
  //Color(255, 0, 0, 0)
  pbitmap->SetPixel(x, y, Color::Red);
}