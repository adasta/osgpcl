/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
*/

#include <osgpcl/pagedlod.h>
#include <osg/CullStack>
#include <osg/Notify>
#include <osgpcl/outofcore_octree_reader.h>
#include <algorithm>

using namespace osg;


osgPCL::PagedLOD::PagedLOD(const PagedLOD& plod,const CopyOp& copyop):
    osg::PagedLOD(plod,copyop), _dboptions(plod._dboptions)
{
  _dboptions.resize(10);
}


namespace osgPCL{
void printBB( std::ostream& cout , osgPCL::OutofCoreOctreeReader::OutOfCoreOptions& opts);
}

void osgPCL::PagedLOD::traverse(NodeVisitor& nv)
{

  // set the frame number of the traversal so that external nodes can find out how active this
  // node is.
  if (nv.getFrameStamp() &&
      nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR)
  {
      setFrameNumberOfLastTraversal(nv.getFrameStamp()->getFrameNumber());
  }

  double timeStamp = nv.getFrameStamp()?nv.getFrameStamp()->getReferenceTime():0.0;
  unsigned int frameNumber = nv.getFrameStamp()?nv.getFrameStamp()->getFrameNumber():0;
  bool updateTimeStamp = nv.getVisitorType()==osg::NodeVisitor::CULL_VISITOR;

  switch(nv.getTraversalMode())
  {
      case(NodeVisitor::TRAVERSE_ALL_CHILDREN):
          std::for_each(_children.begin(),_children.end(),NodeAcceptOp(nv));
          break;
      case(NodeVisitor::TRAVERSE_ACTIVE_CHILDREN):
      {
          float required_range = 0;
          if (_rangeMode==DISTANCE_FROM_EYE_POINT)
          {
              required_range = nv.getDistanceToViewPoint(getCenter(),true);
          }
          else
          {
              osg::CullStack* cullStack = dynamic_cast<osg::CullStack*>(&nv);
              if (cullStack && cullStack->getLODScale()>0.0f)
              {
                  required_range = cullStack->clampedPixelSize(getBound()) / cullStack->getLODScale();
              }
              else
              {
                  // fallback to selecting the highest res tile by
                  // finding out the max range
                  for(unsigned int i=0;i<_rangeList.size();++i)
                  {
                      required_range = osg::maximum(required_range,_rangeList[i].first);
                  }
              }
          }

          int lastChildTraversed = -1;
          bool needToLoadChild = false;
          std::vector<int > idx2load;
          for(unsigned int i=0;i<_rangeList.size();++i)
          {
              if (_rangeList[i].first<=required_range && required_range<_rangeList[i].second)
              {
                  if (i<_children.size())
                  {
                      if (updateTimeStamp)
                      {
                          _perRangeDataList[i]._timeStamp=timeStamp;
                          _perRangeDataList[i]._frameNumber=frameNumber;
                      }

                      _children[i]->accept(nv);
                      lastChildTraversed = (int)i;
                  }
                  else
                  {
                      needToLoadChild = true;

                      unsigned int numChildren = _children.size();
                       // select the last valid child.
                       if (numChildren>0 && ((int)numChildren-1)!=lastChildTraversed)
                       {
                           if (updateTimeStamp)
                           {
                               _perRangeDataList[numChildren-1]._timeStamp=timeStamp;
                               _perRangeDataList[numChildren-1]._frameNumber=frameNumber;
                           }
                           _children[numChildren-1]->accept(nv);
                       }

                      // now request the loading of the next unloaded child.
                      if (!_disableExternalChildrenPaging &&
                          nv.getDatabaseRequestHandler() &&
                          i<_perRangeDataList.size())
                      {
                          // compute priority from where abouts in the required range the distance falls.
                          float priority = (_rangeList[i].second-required_range)/(_rangeList[i].second-_rangeList[i].first);

                          // invert priority for PIXEL_SIZE_ON_SCREEN mode
                          if(_rangeMode==PIXEL_SIZE_ON_SCREEN)
                          {
                              priority = -priority;
                          }

                          // modify the priority according to the child's priority offset and scale.
                          priority = _perRangeDataList[i]._priorityOffset + priority * _perRangeDataList[i]._priorityScale;


                          if (_databasePath.empty())
                          {
                              nv.getDatabaseRequestHandler()->requestNodeFile(_perRangeDataList[i]._filename,nv.getNodePath(),priority,nv.getFrameStamp(), _perRangeDataList[i]._databaseRequest,_dboptions[i].get() );
                          }
                          else
                          {
                              // prepend the databasePath to the child's filename.
                              nv.getDatabaseRequestHandler()->requestNodeFile(_databasePath+_perRangeDataList[i]._filename,nv.getNodePath(),priority,nv.getFrameStamp(), _perRangeDataList[i]._databaseRequest,_dboptions[i].get());
                          }
                      }
                  }
              }
          }

         break;
      }
      default:
          break;
  }
}



bool osgPCL::PagedLOD::addChild(float min, float max,const std::string& filename, osgDB::Options* options, float priorityOffset, float priorityScale)
{

  int  idx = _perRangeDataList.size();
   expandPerRangeDataTo(idx);
   if (idx >= _rangeList.size() ) _rangeList.resize(idx+1);
  _perRangeDataList[idx]._filename=filename;
  _perRangeDataList[idx]._priorityOffset = priorityOffset;
  _perRangeDataList[idx]._priorityScale = priorityScale;
   setChildOptions(idx, options);
  _rangeList[idx].first = min;
  _rangeList[idx].second = max;

}

osgPCL::PagedLOD::PagedLOD () : osg::PagedLOD()
{
  _dboptions.resize(10);
}

void osgPCL::PagedLOD::setChildOptions (int idx, osgDB::Options* options)
{
  int s = _dboptions.size();
  if (idx >= s){
    _dboptions.resize(idx+1);
  }
    _dboptions[idx] = options;
}
