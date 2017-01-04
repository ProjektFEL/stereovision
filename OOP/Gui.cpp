#include "Gui.h"

Gui::Gui()
{};

Gui::Gui(DisparityAlghoritm disparityAlghoritm)
{
	this->disparityAlghoritm = disparityAlghoritm;
};

Gui::~Gui()
{};

void Gui::showDisparitySettings()
{
	int vmin = disparityAlghoritm.getVMin(); int vmax = disparityAlghoritm.getVMax(), smin = disparityAlghoritm.getSMin(),
		mdip = disparityAlghoritm.getMinDisparity(), sp1 = disparityAlghoritm.getP1(), sp2 = disparityAlghoritm.getP2(),
		dmd = disparityAlghoritm.getDispMaxDiff(), pfc = disparityAlghoritm.getPreFilterCap(), sws = disparityAlghoritm.getSpeckleWindowSize(), 
		ssr = disparityAlghoritm.getSpeckleRange(), bsiz = disparityAlghoritm.getBlockSize();          //  premenne pre stereosgbm

	namedWindow("StereoBM control", 0);
	createTrackbar("Vmin", "StereoBM control",  &vmin, 99, 0);
	createTrackbar("Vmax", "StereoBM control", &vmax, 99, 0);
	createTrackbar("Smin", "StereoBM control", &smin, 30, 0);
	createTrackbar("mdip", "StereoBM control", &mdip, 99, 0);
	createTrackbar("dmd", "StereoBM control", &dmd, 99, 0);
	createTrackbar("bsiz", "StereoBM control", &bsiz, 99, 0);
	createTrackbar("sp1", "StereoBM control", &sp1, 1000, 0);
	createTrackbar("sp2", "StereoBM control", &sp2, 2000, 0);
	createTrackbar("pfc", "StereoBM control", &pfc, 200, 0);
	createTrackbar("sws", "StereoBM control", &sws, 200, 0);
	createTrackbar("ssr", "StereoBM control", &ssr, 30, 0);

	disparityAlghoritm.setVMin(vmin);
};

void Gui::showFiltersSettings()
{};