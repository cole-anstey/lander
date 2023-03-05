
// test.cpp : Defines the entry point for the application.
//

#include "stdafx.h"
#include "lander.h"

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <sys/timeb.h>
#include <string.h>
#include <Uxtheme.h>
#include <dwmapi.h>

#include "graphics.h"
#include "world.h"

static int screenphywidth = SCREENWIDTH;
static int screenphyheight = SCREENHEIGHT;
static int screenscale = 2;

#pragma region Main
#define TIMER_ID          1
#define MAX_LOADSTRING    100

bool InitialiseApp(LPTSTR lpCmdLine);
void SetScale(LPTSTR lpCmdLine);
int Repaint(HWND hWnd, int screen_width, int screen_height);
void ExitApp();

#define FRAMETIMEOTHER 1000
unsigned int ulGameFrameTime = FRAMETIMEOTHER;  // 1000 / FPS (xx)
unsigned long ulCurrentFrame, ulPreviousFrame = 0;

/* --- city information --- */
#pragma region

// Global Variables:
HINSTANCE _hInstance;								// current instance
HMODULE _hModule;
HWND _hWnd;
HMENU _hMenu;
TCHAR _szTitle[MAX_LOADSTRING];					// The title bar text
TCHAR _szWindowClass[MAX_LOADSTRING];			// the main window class name

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);

int APIENTRY wWinMain(
  HINSTANCE hInstance,
  HINSTANCE hPrevInstance,
  LPTSTR    lpCmdLine,
  int       nCmdShow)
{
  UNREFERENCED_PARAMETER(hPrevInstance);
  UNREFERENCED_PARAMETER(lpCmdLine);

  MSG msg;
  HACCEL hAccelTable;

  _hInstance = hInstance; // Store instance handle in our global variable   

  // Initialize global strings
  ::LoadString(hInstance, IDS_APP_TITLE, _szTitle, MAX_LOADSTRING);
  ::LoadString(hInstance, IDC_MENU, _szWindowClass, MAX_LOADSTRING);
  MyRegisterClass(hInstance);

  // Initialize GDI+
  GdiplusStartupInput gdiplusStartupInput;
  ULONG_PTR gdiplusToken;

  ::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);

  if (!InitialiseApp(lpCmdLine))
  {
    return false;
  }

  // Perform application initialization:
  if (!InitInstance(hInstance, nCmdShow))
  {
    return false;
  }

  hAccelTable = ::LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_MENU));

  // Main message loop:
  while (::GetMessage(&msg, NULL, 0, 0))
  {
    // Everything is done via the TIMER.
    if (!::TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }
  }

  ::KillTimer(_hWnd, TIMER_ID);

  ExitApp();

  // Destroy GDI+
  ::GdiplusShutdown(gdiplusToken);

  return (int)msg.wParam;
}

//
//  FUNCTION: MyRegisterClass()
//
//  PURPOSE: Registers the window class.
//
//  COMMENTS:
//
//    This function and its usage are only necessary if you want this code
//    to be compatible with Win32 systems prior to the 'RegisterClassEx'
//    function that was added to Windows 95. It is important to call this function
//    so that the application will get 'well formed' small icons associated
//    with it.
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
  WNDCLASSEX wcex;

  wcex.cbSize = sizeof(WNDCLASSEX);

  wcex.style = CS_HREDRAW | CS_VREDRAW;
  wcex.lpfnWndProc = WndProc;
  wcex.cbClsExtra = 0;
  wcex.cbWndExtra = 0;
  wcex.hInstance = hInstance;
  wcex.hIcon = ::LoadIcon(hInstance, MAKEINTRESOURCE(IDI_SMALL));

  BYTE CursorMaskAND[] = { 0xFF };
  BYTE CursorMaskXOR[] = { 0x00 };
  wcex.hCursor = ::CreateCursor(NULL, 0, 0, 1, 1, CursorMaskAND, CursorMaskXOR);
  //wcex.hCursor = LoadCursor(NULL, IDC_ARROW);

  wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
  wcex.lpszMenuName = MAKEINTRESOURCE(IDC_MENU);
  //wcex.lpszMenuName = NULL;
  wcex.lpszClassName = _szWindowClass;
  wcex.hIconSm = ::LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

  return ::RegisterClassEx(&wcex);
}

//
//   FUNCTION: InitInstance(HINSTANCE, int)
//
//   PURPOSE: Saves instance handle and creates main window
//
//   COMMENTS:
//
//        In this function, we save the instance handle in a global variable and
//        create and display the main program window.
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
  HWND hWnd;

  RECT rc = { 0, 0, screenphywidth * screenscale, screenphyheight * screenscale };
  ::AdjustWindowRect(&rc, WS_CAPTION | WS_SYSMENU, false);

  INT nWidth = rc.right - rc.left;
  INT nHeight = rc.bottom - rc.top;

  hWnd = ::CreateWindow(
    _szWindowClass,
    _szTitle,
    WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU,
    CW_USEDEFAULT, 0,
    nWidth,
    nHeight,
    NULL,
    NULL,
    hInstance,
    NULL);

  if (!hWnd)
  {
    return false;
  }

  //BOOL bUseDarkMode = true;
  //BOOL success = SUCCEEDED(::DwmSetWindowAttribute(
  //  hWnd, DWMWINDOWATTRIBUTE::DWMWA_USE_IMMERSIVE_DARK_MODE, &bUseDarkMode, sizeof(use_dark_mode)));

  ::ShowWindow(hWnd, nCmdShow);
  ::UpdateWindow(hWnd);

  //::SetWindowPos(hWnd, HWND_NOTOPMOST, -1, -1, 500, 500, SWP_NOMOVE | SWP_NOOWNERZORDER);

  // Repaint every second.
  ::SetTimer(hWnd, TIMER_ID, 1/*ms*/, NULL);

  _hWnd = hWnd;
  _hMenu = ::GetMenu(hWnd);

  return true;
}

//
//  FUNCTION: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  PURPOSE:  Processes messages for the main window.
//
//  WM_COMMAND	- process the application menu
//  WM_PAINT	- Paint the main window
//  WM_DESTROY	- post a quit message and return
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  int wmId, wmEvent;
  PAINTSTRUCT ps;
  HDC hdc;
  DWORD dwFlags;

  switch (message)
  {
  case WM_TIMER:
    if (wParam == TIMER_ID)
    {
      Repaint(_hWnd, screenphywidth, screenphyheight);
    }

    break;
  case WM_KEYDOWN:
    switch (wParam)
    {
    case VK_F1:
      if (_hMenu) {
        // Toggle the menu.
        bool bMenuVisible = ::GetMenu(hWnd);
        bMenuVisible ? ::SetMenu(hWnd, NULL) : ::SetMenu(hWnd, _hMenu);

        // Resize window after the change in NC space used.
        RECT lpRect;
        ::GetWindowRect(hWnd, &lpRect);

        int nMenuHeight = GetSystemMetrics(SM_CYMENU);

        ::MoveWindow(
          hWnd,
          lpRect.left,
          lpRect.top,
          lpRect.right - lpRect.left,
          (lpRect.bottom - lpRect.top) + (bMenuVisible ? -nMenuHeight : nMenuHeight),
          true
        );
      }
      break;
    case VK_ESCAPE:
      ::PostQuitMessage(0);
      break;
    default:
      break;
    }

    break;
  case WM_SYSKEYDOWN:
    dwFlags = (DWORD)lParam;

    if ((wParam == 0x41/*A*/) &&
      (0x20000000 & dwFlags))
    {
      DialogBox(_hInstance, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
    }

    break;
  case WM_COMMAND:
    wmId = LOWORD(wParam);
    wmEvent = HIWORD(wParam);
    // Parse the menu selections:
    switch (wmId)
    {
    case IDM_ABOUT:
      DialogBox(_hInstance, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);

      break;
    case IDM_EXIT:
      ::DestroyWindow(hWnd);

      break;
    default:
      return DefWindowProc(hWnd, message, wParam, lParam);
    }
    break;
  case WM_SIZE:
    /* --- Free background if we created it --- */
    if (_bmpScreen)
    {
      delete _bmpScreen;
    }

    /* --- Create a new pixmap with new size --- */
    _bmpScreen = new Bitmap(screenphywidth, screenphyheight);
    break;
  case WM_ERASEBKGND:
    return false;
  case WM_PAINT:
  {
    hdc = ::BeginPaint(hWnd, &ps);

    Graphics* g = Graphics::FromHWND(hWnd);

    StretchImage(g, _bmpScreen, 0, 0, screenphywidth, screenphyheight, screenscale);

    delete g;

    ::EndPaint(hWnd, &ps);
  }

  break;
  case WM_DESTROY:
    ::PostQuitMessage(0);

    break;
    //case WM_CLOSE:
    //	if (MessageBox(hWnd, L"Really quit?", L"My application", MB_OKCANCEL) == IDOK)
    //	{
    //		DestroyWindow(hWnd);
    //	}
    //	break;
  default:
    return DefWindowProc(hWnd, message, wParam, lParam);
  }

  return 0;
}

// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
  UNREFERENCED_PARAMETER(lParam);
  switch (message)
  {
  case WM_INITDIALOG:
    return (INT_PTR)true;

  case WM_COMMAND:
    if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
    {
      ::EndDialog(hDlg, LOWORD(wParam));

      return (INT_PTR)true;
    }

    break;
  }

  return (INT_PTR)false;
}

#pragma region Main
#define TITLEFLASHPERIOD    3
#define TITLEFLASH          ((TITLEFLASHPERIOD * 7 /* colours */) * 8 /* iterations */)

// clock ticks
static int nCounter = -1;
static int nCounterTick = 0;
static int nCounterDec = TITLEFLASH;

void UpdateCounters()
{
  struct timeb sTime;

  ftime(&sTime);

  nCounter++;

  if (nCounter > 255)
    nCounter = 0;

  nCounterTick = sTime.millitm < 500;

  if (_bmpScreen > 0)
    nCounterDec--;
}

UINT Milliseconds()
{
  struct timeb sCurrentTime;

  ftime(&sCurrentTime);

  return ((UINT)sCurrentTime.time * 1000) + sCurrentTime.millitm;
}

bool InitialiseApp(LPTSTR lpCmdLine)
{
  _hModule = ::GetModuleHandle(NULL);

  SetScale(lpCmdLine);

  InitialiseImages(_hModule);

  return true;
}

void ExitApp()
{
  // Clean up.
}

/*
 * Repaint
 *
 * data - widget to repaint
 */
int Repaint(HWND hWnd, int screen_width, int screen_height)
{
  // This will render the game at a FPS setting I've decided based on your current hardware,
  // thus the game speed is constant.  A year from now, that frame rate will probably seem rather slow.
  // And your game will happily ignore any advances in technology and graphics hardware.
  // It will crawl on in the same refresh rate it always did.  (But at least with this approach the game is
  // still playable, even if no longer perceived as "smooth" or, at an older computer, the game loop will probably
  // take more than kGameFrameTime ms. The game will run in slow-motion here since it will render every frame no
  // matter what.
  //while (true)
  {
    ulCurrentFrame = Milliseconds();

    ulGameFrameTime = FRAMETIMEOTHER;  // 1000 / FPS (xx)

    if (ulCurrentFrame > (ulPreviousFrame + ulGameFrameTime))
    {
      UpdateCounters();

      Graphics* g = Graphics::FromImage(_bmpScreen);

      g->SetInterpolationMode(InterpolationModeNearestNeighbor);  // No blur

      SolidBrush* blackBrush = new SolidBrush(Color::Black);
      g->FillRectangle(blackBrush, 0, 0, screen_width, screen_height);
      delete blackBrush;

      SetPixel(g, _bmpScreen, 0, 0);

      delete g;

      ulPreviousFrame = ulCurrentFrame;

      ::InvalidateRect(hWnd, NULL, false);

      //break;
    }
  }

  PlayCurrentSound();

  return (true);
}

void SetScale(LPTSTR lpCmdLine)
{
  LPTSTR* szargv;
  int argc;

  szargv = ::CommandLineToArgvW(lpCmdLine, &argc);

  if (szargv &&
    (argc > 1))
  {
    LPTSTR scale = szargv[argc - 1];

    // set game resolution
    if (_tcscmp(scale, L"-1") == 0)
    {
      screenscale = 1;
    }
    else if (_tcscmp(scale, L"-2") == 0)
    {
      screenscale = 2;
    }
    else if (_tcscmp(scale, L"-3") == 0)
    {
      screenscale = 3;
    }
    else if (_tcscmp(scale, L"-help") == 0)
    {
      printf("Cole Anstey 2023.\n");
      printf("lander -help\n");
      printf("lander -2\n\n");
      printf("screen resolutions :\n");
      printf("-1 = 256x176 (default)\n");
      printf("-2 = 512x352\n");
      printf("-3 = 1024x704\n");

      exit(0);
    }
    else
    {
      printf("unknown option\n");
    }

    screenphywidth = SCREENWIDTH;
    screenphyheight = SCREENHEIGHT;
  }

  ::LocalFree(szargv);
}
#pragma region

#pragma region Keyboard
BOOL AnyKeyPressed()
{
  BOOL bPressed = false;

  // Iterate through all key codes to check if *any* key is pressed.
  for (int i = 0; i < 255; i++)
  {
    if (::GetKeyState(i) & 0x80)
    {
      bPressed = true;

      break;
    }
  }

  return bPressed;
}

BOOL IsKeyPressed(int mask)
{
  return (::GetKeyState(mask) & 0x80);
}
#pragma region


void PlayCurrentSound()
{
}