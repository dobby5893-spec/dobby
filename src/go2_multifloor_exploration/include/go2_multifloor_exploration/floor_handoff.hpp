#pragma once
#include <cmath>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <set>
namespace go2_multifloor_exploration {
struct FloorExhaustion {double floor_z,stamp,remaining_unknown_area;unsigned attempts;double deferred_unknown_area{0};double transition_unknown_area{0};};
inline std::optional<FloorExhaustion> parseFloorExhaustion(const std::string & text) {
 std::istringstream input(text);std::string token;
 if(!std::getline(input,token,';') || (token!="v1" && token!="v2" && token!="v3" && token!="v4"))return std::nullopt;
 const bool runtime=token=="v4";
 const bool routes=token=="v3" || runtime;
 const bool regional=token=="v2" || routes;
 std::map<std::string,std::string> fields;
 while(std::getline(input,token,';')) {
  auto eq=token.find('=');if(eq==std::string::npos || !fields.emplace(token.substr(0,eq),token.substr(eq+1)).second)return std::nullopt;
 }
 // Global timeouts/failure counts do not prove that each remaining region
 // is inaccessible. Reject legacy residual deferrals even from an old explorer.
 if(fields["reason"]!=(regional?"region_observability_audit":"closed_regions_excluded"))return std::nullopt;
 try {
  auto number=[&](const std::string & name) {size_t end=0;const auto & value=fields.at(name);
    double result=std::stod(value,&end);if(end!=value.size() || !std::isfinite(result))throw 1;return result;};
  const double z=number("floor_z"),stamp=number("stamp"),area=number("remaining_unknown_area"),attempts=number("attempts");
  const double actionable=number("actionable_unknown_area"),excluded=number("excluded_unknown_area");
  double deferred=0.,transition=0.;
  if(regional) {
    deferred=number("deferred_unknown_area");
    if(routes) {
      transition=number("transition_unknown_area");
      if(transition<0)return std::nullopt;
      if(runtime) {
        if(number("runtime_evidence")!=1 || number("runtime_min_positions")<3 ||
           number("runtime_min_healthy_s")<60 || (number("complete_routes")!=0 && number("complete_routes")!=1))return std::nullopt;
      } else if(number("complete_routes")!=1)return std::nullopt;
    }
    const double count=number("region_count");
    if(actionable!=0 || number("unresolved_unknown_area")!=0 || excluded<0 || deferred<0 || deferred+transition<=0 ||
       std::abs(excluded+deferred+transition-area)>1e-4 ||
       fields["scope"]!="per_region_current_floor" || number("safe_start")!=1 ||
       number("complete_views")!=1 || number("stable_samples")<3 ||
       number("stable_seconds")<10 || count<1 || count>10000 || std::floor(count)!=count)return std::nullopt;
    std::istringstream records(fields.at("regions"));std::string record;std::set<std::string> ids;
    double total=0.,transition_total=0.;bool verification=false;
    while(std::getline(records,record,',')) {
      auto first=record.find(':'),second=record.find(':',first==std::string::npos?0:first+1);
      if(first==std::string::npos || second==std::string::npos || first==0 ||
         record.find(':',second+1)!=std::string::npos)return std::nullopt;
      auto id=record.substr(0,first),reason=record.substr(second+1);
      if(id.find_first_not_of("0123456789")!=std::string::npos || !ids.insert(id).second ||
         (reason!="occluded" && reason!="no_known_access" && !(routes && reason=="no_route") && !(routes && reason=="transition") && !(runtime && (reason=="observed_no_gain" || reason=="verification_deferred"))))return std::nullopt;
      size_t end=0;auto text=record.substr(first+1,second-first-1);double value=std::stod(text,&end);
      if(end!=text.size() || !std::isfinite(value) || value<=0)return std::nullopt;
      if(reason=="verification_deferred")verification=true;
      if(reason=="transition")transition_total+=value;else total+=value;
    }
    if(runtime && !verification && number("complete_routes")!=1)return std::nullopt;
    if(ids.size()!=size_t(count) || std::abs(total-deferred)>1e-4 || std::abs(transition_total-transition)>1e-4)return std::nullopt;
  } else {
    if(actionable!=0. || excluded<=0. || std::abs(excluded-area)>1e-5)return std::nullopt;
    if(fields.count("deferred_unknown_area") && number("deferred_unknown_area")!=0.)return std::nullopt;
  }
  if(stamp<0 || area<0 || attempts<0 || attempts>1000000 || std::floor(attempts)!=attempts)return std::nullopt;
  return FloorExhaustion{z,stamp,area,static_cast<unsigned>(attempts),deferred,transition};
 }catch(...){return std::nullopt;}
}
inline bool matchesActiveFloor(const FloorExhaustion & report,double floor_z,double now) {
 return std::isfinite(floor_z) && std::isfinite(now) && std::abs(report.floor_z-floor_z)<=.15 &&
   now-report.stamp>=-.1 && now-report.stamp<=5.;
}
}
