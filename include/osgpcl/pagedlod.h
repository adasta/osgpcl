/*
 * pagedlod.h
 *
 *  Created on: Aug 4, 2012
 *      Author: Adam Stambler
 */

#ifndef PAGEDLOD_H_
#define PAGEDLOD_H_
#include <osg/PagedLOD>
#include <osgDB/Options>


namespace osgPCL
{

  class OSG_EXPORT PagedLOD : public osg::PagedLOD
  {
      public :

          PagedLOD();

          /** Copy constructor using CopyOp to manage deep vs shallow copy.*/
          PagedLOD(const PagedLOD&,const osg::CopyOp& copyop=osg::CopyOp::SHALLOW_COPY);

          META_Node(osgPCL, PagedLOD);

          virtual void traverse(osg::NodeVisitor& nv);

          using osg::PagedLOD::addChild;

          virtual bool addChild(float min, float max,
              const std::string& filename, osgDB::Options* options = new osgDB::Options(), float priorityOffset=0.0f, float priorityScale=1.0f);


          void setChildOptions( int idx, osgDB::Options* options);

      protected :

          std::vector< osg::ref_ptr<Referenced> > _dboptions;

  };

} /* namespace osgPCL */
#endif /* PAGEDLOD_H_ */
