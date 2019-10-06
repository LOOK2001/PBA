//-------------------------------------------------------
//
//  DynamicalState.C
//
//  Container for data associated with the dynamics
//  degrees of freedom in a system.
//
//  Copyright (c) 2017 Jerry Tessendorf
//
//
//--------------------------------------------------------

#include "DynamicalState.h"
#include <iostream>

using namespace pba;


DynamicalStateData::DynamicalStateData(const std::string& nam) :
  t(0.0),
  nb_items (0),
  name (nam)
{
   // create standard set of attributes:
   //   id, pos, vel, accel, ci, mass
   vector_attributes["pos"] = DSAttribute<Vector>( "pos", Vector(0,0,0) );
   vector_attributes["vel"] = DSAttribute<Vector>( "vel", Vector(0,0,0) );
   vector_attributes["accel"] = DSAttribute<Vector>( "accel", Vector(0,0,0) );
   float_attributes["mass"] = DSAttribute<float>( "mass", 1.0 );
   int_attributes["id"] = DSAttribute<int>( "id", -1 );
   color_attributes["ci"] = DSAttribute<Color>( "ci", Color(1,1,1,0) );
   re_find_main_attrs();
}


DynamicalStateData::DynamicalStateData( const DynamicalStateData& d ) :
  t (d.t),
  nb_items (d.nb_items),
  name (d.name)
{
   int_attributes = d.int_attributes;
   float_attributes = d.float_attributes;
   vector_attributes = d.vector_attributes;
   color_attributes = d.color_attributes;
   re_find_main_attrs();
}


DynamicalStateData::~DynamicalStateData(){}


DynamicalStateData& DynamicalStateData::operator= ( const DynamicalStateData& d )
{
   t = d.t;
   nb_items = d.nb_items;
   name = d.name;
   int_attributes = d.int_attributes;
   float_attributes = d.float_attributes;
   vector_attributes = d.vector_attributes;
   color_attributes = d.color_attributes;
   re_find_main_attrs();
   return *this;
}


void DynamicalStateData::create_attr( const std::string& nam, const int& def )
{
   if( int_attributes.find(nam) != int_attributes.end() ){ return; }
   int_attributes[nam] = DSAttribute<int>( nam, def );
   int_attributes[nam].expand_to( int_attributes["id"].size() );
   re_find_main_attrs();
}

void DynamicalStateData::create_attr( const std::string& nam, const float& def )
{
   if( float_attributes.find(nam) != float_attributes.end() ){ return; }
   float_attributes[nam] = DSAttribute<float>( nam, def );
   float_attributes[nam].expand_to( int_attributes["id"].size() );
   re_find_main_attrs();
}

void DynamicalStateData::create_attr( const std::string& nam, const Vector& def )
{
   if( vector_attributes.find(nam) != vector_attributes.end() ){ return; }
   vector_attributes[nam] = DSAttribute<Vector>( nam, def );
   vector_attributes[nam].expand_to( int_attributes["id"].size() );
   re_find_main_attrs();
}

void DynamicalStateData::create_attr( const std::string& nam, const Color& def )
{
   if( color_attributes.find(nam) != color_attributes.end() ){ return; }
   color_attributes[nam] = DSAttribute<Color>( nam, def );
   color_attributes[nam].expand_to( int_attributes["id"].size() );
   re_find_main_attrs();
}



const size_t DynamicalStateData::add()
{
   size_t add_size = nb_items + 1;
   for( std::map<std::string,DSAttribute<int> >::iterator a = int_attributes.begin(); a != int_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<float> >::iterator a = float_attributes.begin(); a != float_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<Vector> >::iterator a = vector_attributes.begin(); a != vector_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<Color> >::iterator a = color_attributes.begin(); a != color_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   nb_items = nb_items + 1;
   re_find_main_attrs();
   return add_size-1; // return the index of the new particle
}

const size_t DynamicalStateData::add( const size_t nb )
{
   size_t add_size = nb_items + nb;
   for( std::map<std::string,DSAttribute<int> >::iterator a = int_attributes.begin(); a != int_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<float> >::iterator a = float_attributes.begin(); a != float_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<Vector> >::iterator a = vector_attributes.begin(); a != vector_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   for( std::map<std::string,DSAttribute<Color> >::iterator a = color_attributes.begin(); a != color_attributes.end(); a++ )
   {
      a->second.expand_to(add_size);
   }
   nb_items = nb_items + nb;
   re_find_main_attrs();
   return add_size-1; // return the index of the last particle
}

size_t DynamicalStateData::nb() const { return nb_items; }

const int& DynamicalStateData::get_int_attr( const std::string& nam, const size_t p ) const
{
   std::map<std::string,DSAttribute<int> >::const_iterator a = int_attributes.find(nam);
   return a->second.get(p);
}

const float& DynamicalStateData::get_float_attr( const std::string& nam, const size_t p ) const
{
   std::map<std::string,DSAttribute<float> >::const_iterator a = float_attributes.find(nam);
   return a->second.get(p);
}

const Vector& DynamicalStateData::get_vector_attr( const std::string& nam, const size_t p ) const
{
   std::map<std::string,DSAttribute<Vector> >::const_iterator a = vector_attributes.find(nam);
   return a->second.get(p);
}

const Color& DynamicalStateData::get_color_attr( const std::string& nam, const size_t p ) const
{
   std::map<std::string,DSAttribute<Color> >::const_iterator a = color_attributes.find(nam);
   return a->second.get(p);
}

    
const int& DynamicalStateData::id( const size_t p ) const
{
   return ids->second.get(p);
}

const float& DynamicalStateData::mass( const size_t p ) const
{
   return masses->second.get(p);
}

const Vector& DynamicalStateData::pos( const size_t p ) const
{
   return positions->second.get(p);
}

const Vector& DynamicalStateData::vel( const size_t p ) const
{
   return velocities->second.get(p);
}

const Vector& DynamicalStateData::accel( const size_t p ) const
{
   return accelerations->second.get(p);
}

const Color& DynamicalStateData::ci( const size_t p ) const
{
   return cis->second.get(p);
}


void DynamicalStateData::set_attr( const std::string& nam, const size_t p, const int& value )
{
   int_attributes[nam].set(p, value);
}

void DynamicalStateData::set_attr( const std::string& nam, const size_t p, const float& value )
{
   float_attributes[nam].set(p, value);
}

void DynamicalStateData::set_attr( const std::string& nam, const size_t p, const Vector& value ) 
{
   vector_attributes[nam].set(p, value);
}

void DynamicalStateData::set_attr( const std::string& nam, const size_t p, const Color& value ) 
{
   color_attributes[nam].set(p, value);
}


void DynamicalStateData::set_id( const size_t p, const int& value )
{
   ids->second.set(p, value);
}

void DynamicalStateData::set_pos( const size_t p, const Vector& value )
{
   positions->second.set(p, value);
}

void DynamicalStateData::set_vel( const size_t p, const Vector& value )
{
   velocities->second.set(p, value);
}

void DynamicalStateData::set_accel( const size_t p, const Vector& value )
{
   accelerations->second.set(p, value);
}

void DynamicalStateData::set_ci( const size_t p, const Color& value )
{
   cis->second.set(p, value);
}

void DynamicalStateData::set_mass( const size_t p, const float& value )
{
   masses->second.set(p, value);
}

std::vector<std::string> DynamicalStateData::show_int_attrs() const
{
   std::vector<std::string> keys;
   for( std::map<std::string, DSAttribute<int> >::const_iterator i = int_attributes.begin(); i != int_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   return keys;
}

std::vector<std::string> DynamicalStateData::show_float_attrs() const
{
   std::vector<std::string> keys;
   for( std::map<std::string, DSAttribute<float> >::const_iterator i = float_attributes.begin(); i != float_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   return keys;
}


std::vector<std::string> DynamicalStateData::show_vector_attrs() const
{
   std::vector<std::string> keys;
   for( std::map<std::string, DSAttribute<Vector> >::const_iterator i = vector_attributes.begin(); i != vector_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   return keys;
}

std::vector<std::string> DynamicalStateData::show_color_attrs() const
{
   std::vector<std::string> keys;
   for( std::map<std::string, DSAttribute<Color> >::const_iterator i = color_attributes.begin(); i != color_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   return keys;
}

std::vector<std::string> DynamicalStateData::show_all_attrs() const
{
   std::vector<std::string> keys;
   for( std::map<std::string, DSAttribute<int> >::const_iterator i = int_attributes.begin(); i != int_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   for( std::map<std::string, DSAttribute<float> >::const_iterator i = float_attributes.begin(); i != float_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   for( std::map<std::string, DSAttribute<Vector> >::const_iterator i = vector_attributes.begin(); i != vector_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   for( std::map<std::string, DSAttribute<Color> >::const_iterator i = color_attributes.begin(); i != color_attributes.end(); i++ )
   {
      keys.push_back( i->first ); 
   }
   return keys;
}

bool DynamicalStateData::attr_exists( const std::string& nam ) const
{
   if( int_attributes.find(nam) != int_attributes.end() ){ return true; }
   if( float_attributes.find(nam) != float_attributes.end() ){ return true; }
   if( vector_attributes.find(nam) != vector_attributes.end() ){ return true; }
   if( color_attributes.find(nam) != color_attributes.end() ){ return true; }
   return false;
}



void DynamicalStateData::merge( const DynamicalStateData& g )
{
   size_t next_particle = nb();
   for( std::map< std::string, DSAttribute<int> >::const_iterator a = g.int_attributes.begin(); a != g.int_attributes.end(); a++ )
   {
      create_attr( a->second.attr_name(), a->second.default_value() );
   }
   for( std::map< std::string, DSAttribute<float> >::const_iterator a = g.float_attributes.begin(); a != g.float_attributes.end(); a++ )
   {
      create_attr( a->second.attr_name(), a->second.default_value() );
   }
   for( std::map< std::string, DSAttribute<Vector> >::const_iterator a = g.vector_attributes.begin(); a != g.vector_attributes.end(); a++ )
   {
      create_attr( a->second.attr_name(), a->second.default_value() );
   }
   for( std::map< std::string, DSAttribute<Color> >::const_iterator a = g.color_attributes.begin(); a != g.color_attributes.end(); a++ )
   {
      create_attr( a->second.attr_name(), a->second.default_value() );
   }
   add( g.nb() );
   for( std::map< std::string, DSAttribute<int> >::const_iterator a = g.int_attributes.begin(); a != g.int_attributes.end(); a++ )
   {
      for(size_t i=0;i<g.nb();i++ )
      {
         set_attr( a->first, i+next_particle, g.get_int_attr( a->first, i ) );
      } 
   }
   for( std::map< std::string, DSAttribute<float> >::const_iterator a = g.float_attributes.begin(); a != g.float_attributes.end(); a++ )
   {
      for(size_t i=0;i<g.nb();i++ )
      {
         set_attr( a->first, i+next_particle, g.get_float_attr( a->first, i ) );
      } 
   }
   for( std::map< std::string, DSAttribute<Vector> >::const_iterator a = g.vector_attributes.begin(); a != g.vector_attributes.end(); a++ )
   {
      for(size_t i=0;i<g.nb();i++ )
      {
         set_attr( a->first, i+next_particle, g.get_vector_attr( a->first, i ) );
      } 
   }
   for( std::map< std::string, DSAttribute<Color> >::const_iterator a = g.color_attributes.begin(); a != g.color_attributes.end(); a++ )
   {
      for(size_t i=0;i<g.nb();i++ )
      {
         set_attr( a->first, i+next_particle, g.get_color_attr( a->first, i ) );
      } 
   }

   re_find_main_attrs();
}


void DynamicalStateData::re_find_main_attrs()
{
   positions = vector_attributes.find( "pos" );
   if( positions == vector_attributes.end() )
   {
      std::cout << "ERROR could not find positions\n";
   }
   velocities = vector_attributes.find( "vel" );
   if( velocities == vector_attributes.end() )
   {
      std::cout << "ERROR could not find velocities\n";
   }
   accelerations = vector_attributes.find( "accel" );
   if( accelerations == vector_attributes.end() )
   {
      std::cout << "ERROR could not find accelerations\n";
   }
   masses = float_attributes.find( "mass" );
   if( masses == float_attributes.end() )
   {
      std::cout << "ERROR could not find masses\n";
   }
   ids = int_attributes.find( "id" );
   if( ids == int_attributes.end() )
   {
      std::cout << "ERROR could not find ids\n";
   }
   cis = color_attributes.find( "ci" );
   if( cis == color_attributes.end() )
   {
      std::cout << "ERROR could not find cis\n";
   }
}


pba::DynamicalState pba::CreateDynamicalState( const std::string& nam )
{
   return pba::DynamicalState( new pba::DynamicalStateData(nam) );
}


pba::DynamicalState pba::copy( const DynamicalState d )
{
   pba::DynamicalState newcopy = pba::DynamicalState( new DynamicalStateData(*d) );
   return newcopy;
}
