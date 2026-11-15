#pragma once
#include <cmath>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
namespace frontier_exploration_ros2 {
enum class CompletionStatus { VERIFIED_COMPLETE, VERIFIED_WITH_RESIDUALS, INCOMPLETE, BLOCKED, ERROR };
inline const char * completionStatusName(CompletionStatus s) {
 switch(s) {
 case CompletionStatus::VERIFIED_COMPLETE:return "VERIFIED_COMPLETE";
 case CompletionStatus::VERIFIED_WITH_RESIDUALS:return "VERIFIED_WITH_RESIDUALS";
 case CompletionStatus::INCOMPLETE:return "INCOMPLETE";
 case CompletionStatus::BLOCKED:return "BLOCKED";
 default:return "ERROR";
 }
}
struct VerifiedFloorCompletion {
 double floor_z{0},stamp{0},remaining{0},residual{0},transition{0};
 CompletionStatus status{CompletionStatus::INCOMPLETE};
 std::string records;
};
// Dedicated authorization channel; historical v1-v4 remain diagnostic only.
// This is not the future v5 session/map-generation protocol.
inline std::optional<VerifiedFloorCompletion> parseVerifiedFloorCompletion(const std::string & text) {
 std::istringstream in(text);std::string part;std::map<std::string,std::string> f;
 if(!std::getline(in,part,';') || part!="floor_completion")return {};
 while(std::getline(in,part,';')) {
  auto at=part.find('=');
  if(at==std::string::npos || !f.emplace(part.substr(0,at),part.substr(at+1)).second)return {};
 }
 try {
  auto n=[&](const char * key){size_t end=0;auto v=std::stod(f.at(key),&end);
   if(end!=f.at(key).size() || !std::isfinite(v))throw 1;
   return v;};
  VerifiedFloorCompletion out;
  out.floor_z=n("floor_z");out.stamp=n("stamp");out.remaining=n("remaining");
  out.residual=n("residual");out.transition=n("transition");out.records=f.at("regions");
  if(out.stamp<0 || out.remaining<0 || out.residual<0 || out.transition<0 ||
     std::abs(out.remaining-out.residual-out.transition)>1e-4 ||
     f.at("authority")!="joint_residual_audit" || n("safe_start")!=1 ||
     n("complete_views")!=1 || n("complete_routes")!=1 ||
     n("stable_samples")<3 || n("stable_seconds")<10)return {};
  const double count=n("region_count");if(count<0 || count>10000 || std::floor(count)!=count)return {};
  std::set<std::string> ids;double residual=0,transition=0;
  std::istringstream records(out.records);
  while(std::getline(records,part,',')) {
   auto a=part.find(':'),b=part.find(':',a==std::string::npos?0:a+1);
   if(a==std::string::npos || b==std::string::npos || part.find(':',b+1)!=std::string::npos)return {};
   auto id=part.substr(0,a),reason=part.substr(b+1),area=part.substr(a+1,b-a-1);
   if(id.empty() || id.find_first_not_of("0123456789")!=std::string::npos || !ids.insert(id).second)return {};
   size_t end=0;double v=std::stod(area,&end);
   if(end!=area.size() || !std::isfinite(v) || v<=0)return {};
   if(reason=="transition")transition+=v;
   else if(reason=="verified_enclosed")residual+=v;
   else return {}; // No-gain, deferred validation and blocked access are not completion.
  }
  if(ids.size()!=size_t(count) || std::abs(residual-out.residual)>1e-4 ||
     std::abs(transition-out.transition)>1e-4)return {};
  if(f.at("status")=="VERIFIED_COMPLETE" && out.remaining==0 && count==0)
   out.status=CompletionStatus::VERIFIED_COMPLETE;
  else if(f.at("status")=="VERIFIED_WITH_RESIDUALS" && out.remaining>0 && count>0)
   out.status=CompletionStatus::VERIFIED_WITH_RESIDUALS;
  else return {};
  return out;
 }catch(...){return {};}
}
}
