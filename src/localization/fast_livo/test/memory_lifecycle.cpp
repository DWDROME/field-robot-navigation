// Project lifecycle checks, 2026-10-05. GPL-2.0; links the real front-end libraries.
#include "LIVMapper.h"
#include <filesystem>
#include <future>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <sys/resource.h>
#include <csignal>
#include <sstream>
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+" " #x); } while (0)
using namespace livo_memory;
namespace fs = std::filesystem;
cv::Mat pixels() {
  cv::Mat m(64,64,CV_8UC1);
  for (int y=0;y<64;++y) for(int x=0;x<64;++x) m.at<uchar>(y,x)=(x*7+y*3)%256;
  return m;
}
ImageLimits limits() { ImageLimits l; l.hot_bytes=8192; l.cold_bytes=20000; l.records=4; l.files=4; l.jobs=1; l.job_bytes=4096; l.image_bytes=4096; return l; }
void storeChecks() {
  auto m=pixels(); auto l=limits();
  bool invalid=false; try { auto bad=l; bad.hot_bytes=0; ImageStore s("/tmp",bad); } catch (...) { invalid=true; } CHECK(invalid);
  invalid=false; try { ImageStore s("/this-parent-does-not-exist",l); } catch (...) { invalid=true; } CHECK(invalid);
  ImageStore s("/tmp",l); auto a=s.insert(m); CHECK(a); auto lease=s.acquire(a);
  CHECK(lease && cv::norm(*lease,m,cv::NORM_INF)==0);
  s.cool(); s.waitIdle(); CHECK(s.stats().writes==1 && s.stats().hot_bytes==4096);
  lease.reset(); s.cool(); CHECK(s.stats().hot_bytes==0);
  auto one=std::async(std::launch::async,[&]{return s.acquire(a);});
  auto two=std::async(std::launch::async,[&]{return s.acquire(a);});
  auto p=one.get(),q=two.get(); CHECK(p && p.get()==q.get()); CHECK(s.stats().loads==1);
  CHECK(cv::norm(*p,m,cv::NORM_INF)==0);
  p.reset(); q.reset(); s.cool();
  const auto file=s.directory()+"/"+std::to_string(a->id())+".livo";
  {std::fstream f(file,std::ios::in|std::ios::out|std::ios::binary); f.put('x');}
  CHECK(!s.acquire(a)); CHECK(s.stats().failures>0);
  a.reset(); s.collect(); CHECK(s.stats().records==0 && s.stats().files==0);
  auto b=s.insert(m); CHECK(b);
  const auto collision=s.directory()+"/"+std::to_string(b->id())+".livo";
  {std::ofstream f(collision); f<<"foreign sentinel";}
  s.cool(); s.waitIdle(); CHECK(s.stats().files==0); CHECK(s.acquire(b));
  b.reset(); s.collect(); CHECK(fs::exists(collision));
  {std::ifstream f(collision); std::string content; std::getline(f,content); CHECK(content=="foreign sentinel");}
  auto c=s.insert(m); s.cool(); c.reset(); s.shutdown(); CHECK(s.stats().jobs==0 && s.stats().records==0 && s.stats().hot_bytes==0);
  CHECK(fs::exists(collision));
  auto small=l; small.cold_bytes=1;
  ImageStore full("/tmp",small); auto f=full.insert(m); full.cool(); full.waitIdle();
  CHECK(full.stats().rejected>0 && full.stats().hot_bytes==4096 && full.acquire(f));
  auto f2=full.insert(m); CHECK(f2); CHECK(!full.insert(m));
  auto metadata=l; metadata.records=1; ImageStore oneRecord("/tmp",metadata);
  auto only=oneRecord.insert(m); CHECK(only && !oneRecord.insert(m) && oneRecord.stats().records==1);
  ImageStore queueLimit("/tmp",l); auto qa=queueLimit.insert(m), qb=queueLimit.insert(m);
  queueLimit.cool(); CHECK(queueLimit.stats().jobs<=1 && queueLimit.stats().job_bytes<=4096 && queueLimit.stats().rejected>0);
  qa.reset(); qb.reset(); queueLimit.shutdown(); CHECK(queueLimit.stats().records==0 && queueLimit.stats().files==0);
  ImageStore ioFailure("/tmp",l); auto victim=ioFailure.insert(m);
  struct rlimit saved{}; CHECK(getrlimit(RLIMIT_FSIZE,&saved)==0);
  auto shortFile=saved; shortFile.rlim_cur=512;
  auto oldSignal=std::signal(SIGXFSZ,SIG_IGN);
  CHECK(setrlimit(RLIMIT_FSIZE,&shortFile)==0);
  ioFailure.cool(); ioFailure.waitIdle();
  CHECK(setrlimit(RLIMIT_FSIZE,&saved)==0); std::signal(SIGXFSZ,oldSignal);
  // The file-size fault also applies to redirected test logs.
  std::cout.clear(); std::cerr.clear(); clearerr(stdout); clearerr(stderr);
  CHECK(ioFailure.stats().failures>0 && ioFailure.stats().files==0 && ioFailure.acquire(victim));
  // Image identity and pixel leases safely outlive the facade.
  ImageHandle handle; ImageLease late;
  {ImageStore temp("/tmp",l); handle=temp.insert(m); late=temp.acquire(handle); temp.cool();}
  handle.reset(); CHECK(cv::norm(*late,m,cv::NORM_INF)==0); late.reset();
}
void lioChecks() {
  VoxelMapConfig c{}; c.max_voxel_size_=1; c.max_layer_=0; c.max_points_num_=20; c.layer_init_num_={5}; c.planner_threshold_=0.01; c.sigma_num_=3;
  std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> source;
  source[{20,0,0}]=new VoxelOctoTree(0,0,5,20,0.01);
  {
    VoxelMapManager map(c,source); CHECK(source.empty()); map.max_voxels=2;
    pointWithVar a; a.point_w=V3D(1.2,0.2,0.2); map.UpdateVoxelMap({a});
    livo_memory::unpin(map.voxel_map_);
    std::vector<pointWithVar> query{a}; std::vector<PointToPlane> residual;
    map.BuildResidualListOMP(query,residual);
    CHECK(map.voxel_map_.at({1,0,0})->access.pinned);
    pointWithVar b; b.point_w=V3D(3.2,0.2,0.2); map.UpdateVoxelMap({b});
    CHECK(map.voxel_map_.count({1,0,0}) && !map.voxel_map_.count({20,0,0}));
    auto rejected=map.budget_rejections; b.point_w.x()=5.2; map.UpdateVoxelMap({b});
    CHECK(map.budget_rejections==rejected+1 && map.voxel_map_.size()==2);
    map.clearMemOutOfMap(0,0,0,0,0,0); CHECK(map.voxel_map_.size()==2);
    livo_memory::unpin(map.voxel_map_); map.clearMemOutOfMap(0,0,0,0,0,0); CHECK(map.voxel_map_.empty());
  }
  CHECK(LiveObjects::trees==0 && LiveObjects::planes==0);
}
void configureVio(VIOManager& v, vk::AbstractCamera& cam, StatesGroup& state) {
  v.cam=&cam; v.state=&state; v.state_propagat=&state;
  v.grid_size=16; v.grid_n_height=4; v.patch_size=4; v.patch_pyrimid_level=2;
  v.normal_en=true; v.inverse_composition_en=true; v.raycast_en=false; v.plot_flag=false;
  v.max_iterations=1; v.outlier_threshold=1e9; v.img_point_cov=100;
  v.setImuToLidarExtrinsic(V3D::Zero(),M3D::Identity());
  std::vector<double> rotation{1,0,0,0,1,0,0,0,1}, translation{0,0,0}; v.setLidarToCameraExtrinsic(rotation,translation);
  v.initializeVIO(); v.images.reset(new ImageStore("/tmp",limits()));
  auto m=pixels(); v.new_frame_.reset(new Frame(&cam,m)); v.updateFrameState(state); v.current_image=v.images->insert(m);
}
VisualPoint* addPoint(VIOManager& v, const V3D& position) {
  auto* p=new VisualPoint(position); p->normal_=V3D(0,0,-1); p->is_normal_initialized_=true;
  auto px=v.cam->world2cam(position); auto patch=std::make_unique<float[]>(v.patch_size_total);
  auto image=v.images->acquire(v.current_image); v.getImagePatch(*image,px,patch.get(),0);
  auto* f=new Feature(p,patch.release(),px,v.cam->cam2world(px),v.new_frame_->T_f_w_,0);
  f->img_=v.current_image; f->id_=v.new_frame_->id_; f->inv_expo_time_=1;
  p->addFrameRef(f); v.insertPointIntoVoxelMap(p); return p;
}
void vioChecks() {
  {VIOManager partial;}
  vk::PinholeCamera cam(64,64,1,40,40,32,32); StatesGroup state;
  {
    VIOManager v; configureVio(v,cam,state); auto* point=addPoint(v,V3D(0,0,3));
    v.points_per_voxel=1; addPoint(v,V3D(0.1,0,3)); CHECK(v.visual_points==1 && LiveObjects::features==1);
    v.points_per_voxel=32; v.budget_rejections=0;
    auto m=pixels(); pointWithVar query; query.point_w=point->pos_; std::vector<pointWithVar> points{query};
    std::unordered_map<VOXEL_LOCATION,VoxelOctoTree*> planes;
    v.resetGrid(); v.retrieveFromVisualSparseMap(m,points,planes); CHECK(v.total_points==1);
    auto hot=v.visual_submap->warp_patch; v.precomputeReferencePatches(0); auto jac=v.H_sub_inv;
    for(int i=0;i<100;++i) v.updateVisualMapPoints(m);
    CHECK(point->obs_.size()==1 && LiveObjects::features==1);
    v.releaseBorrowed(); v.images->cool(); v.images->waitIdle(); CHECK(v.images->stats().hot_bytes==0);
    v.resetGrid(); v.retrieveFromVisualSparseMap(m,points,planes); CHECK(v.total_points==1 && hot==v.visual_submap->warp_patch);
    v.precomputeReferencePatches(0); CHECK((jac-v.H_sub_inv).norm()==0);
    v.projectPatchFromRefToCur(planes);
    v.normal_en=false; v.resetGrid(); v.retrieveFromVisualSparseMap(m,points,planes); CHECK(v.total_points==1 && LiveObjects::warps==1);
    v.resetGrid(); v.retrieveFromVisualSparseMap(m,points,planes); CHECK(LiveObjects::warps==1);
    v.precomputeReferencePatches(1); CHECK(v.H_sub_inv.allFinite());
    v.max_voxels=1; v.max_visual_points=1; addPoint(v,V3D(0.6,0,3)); CHECK(v.visual_points==1 && v.budget_rejections==1);
    v.releaseBorrowed(); CHECK(LiveObjects::warps==0);
    v.images->cool(); auto file=v.images->directory()+"/"+std::to_string(v.current_image->id())+".livo"; fs::remove(file);
    v.resetGrid(); v.retrieveFromVisualSparseMap(m,points,planes); CHECK(v.total_points==0 && v.reference_images.empty());
    livo_memory::unpin(v.feat_map); v.current_image=v.images->insert(m);
    addPoint(v,V3D(0.6,0,3)); CHECK(v.lru_evictions==1 && v.visual_points==1);
    livo_memory::unpin(v.feat_map); v.trimVisualMap(V3D(1000,0,0)); CHECK(v.feat_map.empty());
    cv::Mat rgb; cv::cvtColor(m,rgb,cv::COLOR_GRAY2BGR); std::vector<pointWithVar> empty;
    v.processFrame(rgb,empty,planes,0);
    CHECK(v.visual_submap->voxel_points.empty() && v.reference_images.empty() && v.new_frame_ && !v.img_rgb.empty());
  }
  CHECK(LiveObjects::points==0 && LiveObjects::features==0 && LiveObjects::warps==0);
}
void mapperChecks() {
  ros::NodeHandle nh;
  nh.setParam("imu/imu_en",true);
  nh.setParam("extrin_calib/extrinsic_T",std::vector<double>{0,0,0});
  nh.setParam("extrin_calib/Pcl",std::vector<double>{0,0,0});
  nh.setParam("extrin_calib/extrinsic_R",std::vector<double>{1,0,0,0,1,0,0,0,1});
  nh.setParam("extrin_calib/Rcl",std::vector<double>{1,0,0,0,1,0,0,0,1});
  nh.setParam("laserMapping/cam_model",std::string("Pinhole"));
  nh.setParam("laserMapping/cam_width",64); nh.setParam("laserMapping/cam_height",64);
  nh.setParam("laserMapping/cam_fx",40.0); nh.setParam("laserMapping/cam_fy",40.0);
  nh.setParam("laserMapping/cam_cx",32.0); nh.setParam("laserMapping/cam_cy",32.0);
  nh.setParam("vio/grid_size",16); nh.setParam("vio/patch_size",4); nh.setParam("vio/patch_pyrimid_level",2);
  nh.setParam("memory/input_messages",2); nh.setParam("memory/path_poses",3);
  nh.setParam("memory/accumulation_points",4);
  {
    LIVMapper m(nh); image_transport::ImageTransport transport(nh); m.initializeSubscribersAndPublishers(nh,transport);
    m.last_timestamp_lidar=1;
    auto image=boost::make_shared<sensor_msgs::Image>(); image->width=64; image->height=64; image->step=192; image->encoding="bgr8"; image->data.resize(64*192);
    for (int i=0;i<3;++i) {image->header.stamp=ros::Time(1.0+i*0.05); m.img_cbk(image);}
    CHECK(m.img_buffer.size()==1 && m.img_time_buffer.size()==1 && m.input_drops>0);
    m.discardInputEpoch("test reset");
    for (int i=0;i<8;++i) m.publish_path(m.pubPath);
    CHECK(m.path.poses.size()==3 && m.path_drops==5);
    auto mono=pixels(); m.vio_manager->new_frame_.reset(new Frame(m.vio_manager->cam,mono)); m.vio_manager->updateFrameState(m._state);
    m.vio_manager->img_rgb=cv::Mat::zeros(64,64,CV_8UC3);
    m.LidarMeasures.lio_vio_flg=VIO; MeasureGroup g; g.vio_time=2; m.LidarMeasures.measures.push_back(g);
    PointType behind{}; behind.z=-1;
    for(int i=0;i<12;++i) {m.pcl_w_wait_pub->push_back(behind); m.publish_frame_world(m.pubLaserCloudFullRes,m.vio_manager); CHECK(m.pcl_wait_pub->empty());}
    // Actual LIVO splitter rejects an accumulation before reserving/copying it.
    m.LidarMeasures.lio_vio_flg=WAIT; m.LidarMeasures.last_lio_update_time=1;
    auto cloud=boost::make_shared<PointCloudXYZI>(); cloud->resize(5); for(auto& p:cloud->points) p.curvature=1000;
    m.lid_raw_data_buffer.push_back(cloud); m.lid_header_time_buffer.push_back(1);
    m.img_buffer.push_back(cv::Mat::zeros(64,64,CV_8UC3)); m.img_time_buffer.push_back(1.5);
    auto imu=boost::make_shared<sensor_msgs::Imu>(); imu->header.stamp=ros::Time(2.0); m.imu_buffer.push_back(imu);
    CHECK(!m.sync_packages(m.LidarMeasures)); CHECK(m.lid_raw_data_buffer.empty() && m.LidarMeasures.pcl_proc_cur->empty());
    m.p_imu->imu_need_init=false; m._state.pos_end=V3D(1,2,3); g.imu.push_back(imu); g.lio_time=2;
    m.LidarMeasures.measures.push_back(g); m.LidarMeasures.lio_vio_flg=LIO;
    m.p_imu->Process2(m.LidarMeasures,m._state,m.feats_undistort);
    CHECK(m.feats_undistort->empty() && m._state.pos_end==V3D(1,2,3) && m.p_imu->retainedPoses()==0);
    // Empty selected output clouds cause real PCL write failures before opening files.
    // Keep the opposite output nonempty to exercise all three savePCD success logs.
    m.pcd_save_en=true; m.pcd_save_interval=-1; m.colmap_output_en=false;
    m.pcl_wait_save->clear(); m.pcl_wait_save_intensity->clear();
    std::ostringstream captured;
    {
      auto* previous=std::cout.rdbuf(captured.rdbuf());
      ScopeExit restore([&]{std::cout.rdbuf(previous);});
      m.img_en=true; m.pcl_wait_save_intensity->push_back(behind); m.savePCD();
      m.pcl_wait_save_intensity->clear(); m.pcl_wait_save->push_back(pcl::PointXYZRGB{});
      m.img_en=false; m.savePCD();
    }
    CHECK(captured.str().find("saved to:")==std::string::npos);
    m.pcd_save_en=false; m.pcl_wait_save->clear();
    auto h=m.vio_manager->images->insert(mono); m.vio_manager->images->cool();
  }
  CHECK(LiveObjects::points==0 && LiveObjects::features==0 && LiveObjects::trees==0);
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"fastlivo_memory_lifecycle",ros::init_options::AnonymousName|ros::init_options::NoSigintHandler);
  ros::NodeHandle node;
  if (argc > 1 && std::string(argv[1]) == "--dependency-baseline") {
    // No project manager/store objects: isolates process-global dependency retention.
    vk::PinholeCamera camera(64,64,1,40,40,32,32);
    image_transport::ImageTransport transport(node);
    auto publisher=transport.advertise("/dependency_baseline",1);
    auto input=pixels(); cv::Mat output; cv::resize(input,output,cv::Size(128,128));
    std::cout << "dependency baseline only" << std::endl;
    return 0;
  }
  try { storeChecks(); lioChecks(); vioChecks(); mapperChecks(); std::cout<<"store, LIO, VIO and mapper lifecycle checks passed\n"; }
  catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
