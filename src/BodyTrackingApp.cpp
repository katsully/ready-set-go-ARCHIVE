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

#include "cinder/app/App.h"
#include "cinder/params/Params.h"
#include "cinder/gl/gl.h" 
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"

#include "Kinect2.h"

class BodyTrackingApp : public ci::app::App
{
public:
	BodyTrackingApp();

	void						draw() override;
	void						update() override;
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
	std::vector<ci::ColorA8u> shirtColors;
	std::vector<ci::ColorA8u> bodyColors;
	int bodyColorIdx;
	std::map<ci::ColorA8u, ci::ColorA8u> matchColors;
};



using namespace ci;
using namespace ci::app;
using namespace std;

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

	bodyColors.push_back(Color(1, 0, 0));	// red
	bodyColors.push_back(Color(1, 1, 0));	// yellow
	bodyColors.push_back(Color(0, 1, 0));	// green
	bodyColors.push_back(Color(0, 0, 1));	// blue
	bodyColors.push_back(Color(0, 0, 0));	// black
	bodyColors.push_back(Color(1, 1, 1));	// white
	bodyColorIdx = 0;
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
				gl::color(Color(1, 0, 0));
				for (const auto& joint : body.getJointMap()) {
					if (joint.second.getTrackingState() == TrackingState::TrackingState_Tracked) {
						if (joint.first == JointType_SpineMid) {
							const vec2 midSpinePos = mDevice->mapCameraToColor(joint.second.getPosition());
							ColorA8u shirtColor = mSurfaceColor->getPixel(midSpinePos);
							bool newPerson = true;
							int idx = 0;
							for (ColorA8u c : shirtColors) {
								if (abs(shirtColor.r - c.r) < 30 && abs(shirtColor.g - c.g) < 30 && abs(shirtColor.b - c.b) < 30) {
									newPerson = false;
									break;
								}
								idx++;
							}
							if (newPerson && shirtColors.size() < 6) {
								shirtColors.push_back(shirtColor);
								idx = shirtColors.size() - 1;
							}
							if (idx > 5) {
								idx = 0;
							}
							console() << shirtColors.size() << endl;
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

CINDER_APP(BodyTrackingApp, RendererGl, [](App::Settings* settings)
{
	settings->prepareWindow(Window::Format().size(1024, 768).title("Body Tracking App"));
	settings->disableFrameRate();
})
