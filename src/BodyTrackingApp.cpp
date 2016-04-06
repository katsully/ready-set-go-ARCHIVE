/*
*
* Copyright (c) 2015, Wieden+Kennedy
* Stephen Schieberl, Michael Latzoni
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or
* without modification, are permitted provided that the following
* conditions are met:
*
* Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
*
* Neither the name of the Ban the Rewind nor the names of its
* contributors may be used to endorse or promote products
* derived from this software without specific prior written
* permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

// Edited by Kat Sullivan

#include "cinder/app/App.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h" 
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"

#include "Kinect2.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class BodyTrackingApp : public ci::app::App
{
public:
	BodyTrackingApp();

	void						draw() override;
	void						update() override;
	void						keyDown(KeyEvent event);
private:
	Kinect2::BodyFrame			mBodyFrame;
	ci::Channel8uRef			mChannelBodyIndex;
	ci::Channel16uRef			mChannelDepth;
	Kinect2::DeviceRef			mDevice;
	ci::Surface8uRef			mSurfaceColor;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;

	// list of colors representing a person
	vector<ColorA8u> shirtColors;
	vector<ColorA8u> bodyColors;
	vector < tuple<ColorA8u, ColorA8u> > outfits;
};

BodyTrackingApp::BodyTrackingApp()
{
	mFrameRate = 0.0f;
	mFullScreen = false;

	mDevice = Kinect2::Device::create();
	mDevice->start();
	mDevice->connectBodyEventHandler([&](const Kinect2::BodyFrame frame)
	{
		mBodyFrame = frame;
	});
	mDevice->connectBodyIndexEventHandler([&](const Kinect2::BodyIndexFrame frame)
	{
		mChannelBodyIndex = frame.getChannel();
	});
	mDevice->connectDepthEventHandler([&](const Kinect2::DepthFrame frame)
	{
		mChannelDepth = frame.getChannel();
	});
	mDevice->connectColorEventHandler([&](const Kinect2::ColorFrame& frame)
	{
		mSurfaceColor = frame.getSurface();
	});

	mParams = params::InterfaceGl::create("Params", ivec2(200, 100));
	mParams->addParam("Frame rate", &mFrameRate, "", true);
	mParams->addParam("Full screen", &mFullScreen).key("f");
	mParams->addButton("Quit", [&]() { quit(); }, "key=q");
	console() << "HERE" << endl;

	bodyColors.push_back(Color(1, 0, 0));	// red
	bodyColors.push_back(Color(1, 1, 0));	// yellow
	bodyColors.push_back(Color(0, 1, 0));	// green
	bodyColors.push_back(Color(0, 0, 1));	// blue
	bodyColors.push_back(Color(0, 0, 0));	// black
	bodyColors.push_back(Color(1, 1, 1));	// white
}

void BodyTrackingApp::draw()
{
	const gl::ScopedViewport scopedViewport(ivec2(0), getWindowSize());
	const gl::ScopedMatrices scopedMatrices;
	const gl::ScopedBlendAlpha scopedBlendAlpha;
	gl::setMatricesWindow(getWindowSize());
	gl::clear();
	gl::color(ColorAf::white());
	gl::disableDepthRead();
	gl::disableDepthWrite();

	if (mChannelDepth) {
		gl::enable(GL_TEXTURE_2D);
		const gl::TextureRef tex = gl::Texture::create(*Kinect2::channel16To8(mChannelDepth));
		gl::draw(tex, tex->getBounds(), Rectf(getWindowBounds()));
	}

	if (mChannelBodyIndex) {

		gl::enable(GL_TEXTURE_2D);

		auto drawHand = [&](const Kinect2::Body::Hand& hand, const ivec2& pos) -> void
		{
			switch (hand.getState()) {
			case HandState_Closed:
				gl::color(ColorAf(1.0f, 0.0f, 0.0f, 0.5f));
				break;
			case HandState_Lasso:
				gl::color(ColorAf(0.0f, 0.0f, 1.0f, 0.5f));
				break;
			case HandState_Open:
				gl::color(ColorAf(0.0f, 1.0f, 0.0f, 0.5f));
				break;
			default:
				gl::color(ColorAf(0.0f, 0.0f, 0.0f, 0.0f));
				break;
			}
			gl::drawSolidCircle(pos, 30.0f, 32);
		};

		const gl::ScopedModelMatrix scopedModelMatrix;
		gl::scale(vec2(getWindowSize()) / vec2(mChannelBodyIndex->getSize()));
		gl::disable(GL_TEXTURE_2D);
		for (const Kinect2::Body& body : mBodyFrame.getBodies()) {
			if (body.isTracked()) {
				// color for skeleton
				gl::color(Color(1, 0, 1));
				bool newPerson = true;
				ColorA8u shirtColor;
				ColorA8u pantColor;
				for (const auto& joint : body.getJointMap()) {
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						// get the color from the performer's right shoulder
						if (joint.first == JointType_ShoulderRight) {
							const vec2 midSpinePos = mDevice->mapCameraToColor(joint.second.getPosition());
							shirtColor = mSurfaceColor->getPixel(midSpinePos);
							for (ColorA8u c : shirtColors) {
								if (abs(shirtColor.r - c.r) < 45 && abs(shirtColor.g - c.g) < 45 && abs(shirtColor.b - c.b) < 45) {
									newPerson = false;
									break;
								}
							}
							if (newPerson && shirtColors.size() < 6) {
								shirtColors.push_back(shirtColor);
							}
						}
						// second color tracker to look at pant color
						else if (joint.first == JointType_KneeRight) {
							const vec2 kneeRightPos = mDevice->mapCameraToColor(joint.second.getPosition());
							ColorA8u pantColor = mSurfaceColor->getPixel(kneeRightPos);
							int idx = 0;
							
							// if this is possibly an existing person because the shirt matched
							if(!newPerson) {
								for (tuple<ColorA8u, ColorA8u> outfit: outfits) {
									ColorA8u c = std::get<0>(outfit);
									if (abs(shirtColor.r - c.r) < 45 && abs(shirtColor.g - c.g) < 45 && abs(shirtColor.b - c.b) < 45) {
										ColorA8u p = std::get<1>(outfit);
										if (abs(pantColor.r - c.r) < 45 && abs(pantColor.g - c.g) < 45 && abs(pantColor.b - c.b) < 45) {
											newPerson = false;
											break;
										}
										idx++;
									}
								}
							}
							if (newPerson && outfits.size() < 6) {
								outfits.push_back(make_tuple(shirtColor, pantColor));
								idx = outfits.size() - 1;
							}
							if (idx > 6) {
								idx = 0;
							}
							console() << "OUTFITS SIZE: ";
							console() << outfits.size() << endl;
							console() << "INDEX COUNT: ";
							console() << idx << endl;
							gl::color(bodyColors[idx]);
						}
						const vec2 pos(mDevice->mapCameraToDepth(joint.second.getPosition()));
						gl::drawSolidCircle(pos, 5.0f, 32);
						const vec2 parent(mDevice->mapCameraToDepth(
							body.getJointMap().at(joint.second.getParentJoint()).getPosition()
							));
						gl::drawLine(pos, parent);
					}
				}
				drawHand(body.getHandLeft(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandLeft).getPosition()));
				drawHand(body.getHandRight(), mDevice->mapCameraToDepth(body.getJointMap().at(JointType_HandRight).getPosition()));
			}
		}
	}

	mParams->draw();
}

void BodyTrackingApp::update()
{
	mFrameRate = getAverageFps();

	if (mFullScreen != isFullScreen()) {
		setFullScreen(mFullScreen);
		mFullScreen = isFullScreen();
	}
}

void BodyTrackingApp::keyDown(KeyEvent event) {
	if ('a' == event.getChar()) {
		console() << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << endl;
	}
}

CINDER_APP(BodyTrackingApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1024, 768).title("Body Tracking App"));
	settings->disableFrameRate();
})
