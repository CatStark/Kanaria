#pragma once

#ifndef NewStitching_H
#define NewStitching_H
 
#include <OgreRoot.h>
#include <OgreCamera.h>
#include <OgreViewport.h>
#include <OgreSceneManager.h>
#include <OgreRenderWindow.h>
#include <OgreConfigFile.h>
#include <OgreException.h>
#include <OgreEntity.h>
#include <OgreFrameListener.h>
#include <OgreWindowEventUtilities.h>
 
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISMouse.h>

#include <OgreLogManager.h>
#include <SdkTrays.h>
#include <SdkCameraMan.h>
#include <utility>
#include <stdlib.h>

#include "GridCell.h"
 
/*#include <CEGUI/CEGUI.h>
#include <CEGUI/RendererModules/Ogre/Renderer.h*/

class NewStitching: public Ogre::WindowEventListener, public Ogre::FrameListener, public OIS::KeyListener, public OIS::MouseListener,  public OgreBites::SdkTrayListener
{
public:
  NewStitching();
  ~NewStitching();
  std::vector<Ogre::Vector3> m_vertices;														//Four vertices per patch

  void go();
  void displayError(int x, int y, Ogre::Real bestFit);
 
private:
  bool processUnbufferedInput(const Ogre::FrameEvent& fe);
  virtual bool frameRenderingQueued(const Ogre::FrameEvent& fe);
 
  virtual bool keyPressed(const OIS::KeyEvent& ke);
  virtual bool keyReleased(const OIS::KeyEvent& ke);
 
  virtual bool mouseMoved(const OIS::MouseEvent& me);
  virtual bool mousePressed(const OIS::MouseEvent& me, OIS::MouseButtonID id);
  virtual bool mouseReleased(const OIS::MouseEvent& me, OIS::MouseButtonID id);
 
  virtual void windowResized(Ogre::RenderWindow* rw);
  virtual void windowClosed(Ogre::RenderWindow* rw);
 
  bool setup();
  bool configure();
  void chooseSceneManager();
  void createCamera();
  void createScene();
  void destroyScene();
  virtual void createFrameListener(void);
  void createViewports();
  void setupResources();
  void createResourceListener();
  void loadResources();  
  
//New
  void startAnimation();
  void createGrid(int height, int width);
  void createTemplate();
  void createPatches();
  std::pair<int, int> retrieveXY(int x, int y);
  bool clickM; //
  
  void dualViewPort(Ogre::SceneManager *primarySceneMgr, Ogre::SceneManager *secondarySceneMgr);
  
  //Ogre::SceneManager* mSecondarySceneMgr;


  
  
  //Vertices and indices for the rest of patches
  std::vector<Ogre::Vector3> p_vertices;
  std::vector<int> p_indices;

  bool mShutdown;
	
  Ogre::Root* mRoot;
  Ogre::Camera* mCameraFront;
  Ogre::SceneManager* mSceneMgr;
  Ogre::RenderWindow* mWindow;
  Ogre::String mResourcesCfg;
  Ogre::String mPluginsCfg;

   Ogre::SceneManager* mSecondarySceneMgr;
   Ogre::Camera* mCameraSideView;
   Ogre::Camera* mCameraBack;

  

  //Fix for 1.9
	Ogre::OverlaySystem *mOverlaySystem;
	OgreBites::InputContext mInputContext;
  
  // OgreBites
    OgreBites::SdkTrayManager* mTrayMgr;
    OgreBites::SdkCameraMan* mCameraMan;       // basic camera controller
    OgreBites::ParamsPanel* mDetailsPanel;     // sample details panel

  // OIS
  OIS::Mouse* mMouse;
  OIS::Keyboard* mKeyboard;
  OIS::InputManager* mInputMgr;
 

  std::string triangle;
  std::string triangleChild;
  //////////////////////

  bool mouseFlag;
  std::vector<Patch*> _patches;



  Ogre::RaySceneQuery* m_pray_scene_query;
};
 
#endif
