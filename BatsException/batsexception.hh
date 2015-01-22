/*
 *  Little Green BATS (2008-2013), AI department, University of Groningen
 *
 *  Authors: 	Sander van Dijk (sgdijk@ai.rug.nl)
 *		Drew Noakes (drew@drewnoakes.com)
 *		Martin Klomp (martin@ai.rug.nl)
 *		Mart van de Sanden (vdsanden@ai.rug.nl)
 *		A. Bram Neijt (bneijt@gmail.com)
 *		Matthijs Platje (mplatje@gmail.com)
 *		Jeroen Kuijpers (jkuypers@gmail.com)
 *
 *  Date: 	August 17, 20013
 *
 *  Website:	https://github.com/sgvandijk/libbats
 *
 *  Comment:	Please feel free to contact us if you have any 
 *		problems or questions about the code.
 *
 *
 *  License: 	This program is free software; you can redistribute 
 *		it and/or modify it under the terms of the GNU General
 *		Public License as published by the Free Software 
 *		Foundation; either version 3 of the License, or (at 
 *		your option) any later version.
 *
 *   		This program is distributed in the hope that it will
 *		be useful, but WITHOUT ANY WARRANTY; without even the
 *		implied warranty of MERCHANTABILITY or FITNESS FOR A
 *		PARTICULAR PURPOSE.  See the GNU General Public
 *		License for more details.
 *
 *   		You should have received a copy of the GNU General
 *		Public License along with this program; if not, write
 *		to the Free Software Foundation, Inc., 59 Temple Place - 
 *		Suite 330, Boston, MA  02111-1307, USA.
 *
 */
#ifndef BATS_BATSEXCEPTION_HH
#define BATS_BATSEXCEPTION_HH

#include <stdexcept>
#include <sstream>

namespace bats {

  /** Exception base class
   *
   *  This exception class can be used, in companion with the
   *  BATS_CATCH_FRAME, to keep a trace of places where it is thrown
   *  and caught.
   */
  class BatsException : public std::exception {

    std::stringstream d_messages;

    void copy(BatsException const &other)
    {
      // d_messages = other.d_messages;
    }

    void destroy()
    {
    }

    BatsException(BatsException const &); // NI
    BatsException &operator=(BatsException const &); // NI

  public:

    /**
     * Initialize message with a single string
     */
    BatsException(std::string const msg)
    {
      d_messages << msg << std::endl;
    }

    /**
     * Initialize message with name of file and line number where the
     * exception originated and some text
     */
    BatsException(std::string const &filename, unsigned line, std::string const msg)
    {
      d_messages << filename << ":" << line << ": " << msg << std::endl;
    }

    /**
     * Retrieve message stream. New messages can be added to this
     * stream.
     */
    std::stringstream &messages() { return d_messages; }

    /**
     * Retrieve total message
     */
    virtual char const *what() const throw () { return d_messages.str().c_str(); }

    virtual ~BatsException() throw () { destroy(); }

  };


};

/**
 * Add new line with source file and line number to the exception
 * message. Rethrows the exception
 */
#define BATS_CATCH_FRAME(e) e->messages() << __FILE__ << ":" << __LINE__ << std::endl; throw e


#endif // INC_BATS_BATSEXCEPTION_HH
