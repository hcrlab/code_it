#!/usr/bin/env python
from bson.objectid import ObjectId
from pymongo import MongoClient
import datetime
import pymongo
import rospy

from code_it_msgs.msg import Program
from code_it_msgs.srv import AddProgram, AddProgramResponse
from code_it_msgs.srv import CopyProgram, CopyProgramResponse
from code_it_msgs.srv import GetProgram, GetProgramResponse
from code_it_msgs.srv import DeleteProgram, DeleteProgramResponse
from code_it_msgs.srv import UpdateProgram, UpdateProgramResponse
from code_it_msgs.srv import ListPrograms, ListProgramsResponse


class ProgramManager(object):
    def __init__(self, db):
        self._db = db

    def add(self):
        result = self._db.programs.insert_one(
            {'name': 'Untitled program',
             'xml': '<xml></xml>'})
        return self.get(str(result.inserted_id))

    def get(self, id_str):
        return self._db.programs.find_one({'_id': ObjectId(id_str)})

    def delete(self, id_str):
        result = self._db.programs.delete_one({'_id': ObjectId(id_str)})

    def update(self, id_str, mods):
        """Update a program

        id_str: The MongoDB ObjectId as a string.
        mods: A dictionary of fields to modify, e.g., {'name': 'Hello world'}
        """
        mods['last_update'] = datetime.datetime.utcnow()
        result = self._db.programs.update_one({'_id': ObjectId(id_str)},
                                              {'$set': mods})

    def list(self):
        return self._db.programs.find().sort([('last_update',
                                               pymongo.DESCENDING)])


class ProgramServer(object):
    def __init__(self, manager):
        self._manager = manager
        rospy.Service('code_it/add_program', AddProgram, self.handle_add)
        rospy.Service('code_it/copy_program', CopyProgram, self.handle_copy)
        rospy.Service('code_it/get_program', GetProgram, self.handle_get)
        rospy.Service('code_it/delete_program', DeleteProgram,
                      self.handle_delete)
        rospy.Service('code_it/update_program', UpdateProgram,
                      self.handle_update)
        rospy.Service('code_it/list_programs', ListPrograms, self.handle_list)

    def handle_add(self, request):
        program_dict = self._manager.add()
        program = Program()
        program.program_id = str(program_dict['_id'])
        program.name = program_dict['name']
        program.xml = program_dict['xml']
        return AddProgramResponse(program)

    def handle_copy(self, request):
        program_id = request.program_id
        program_dict = self._manager.get(program_id)
        program_name = 'Untitled program'
        if 'name' in program_dict:
            program_name = 'Copy of ' + program_dict['name']
        program_xml = '<xml></xml>'
        if 'xml' in program_dict:
            program_xml = program_dict['xml']

        copy_dict = self._manager.add()
        inserted_id_str = str(copy_dict['_id'])
        self._manager.update(inserted_id_str,
                             {'name': program_name,
                              'xml': program_xml})
        program = Program()
        program.program_id = str(copy_dict['_id'])
        program.name = program_name
        program.xml = program_xml
        return CopyProgramResponse(program)

    def handle_get(self, request):
        program_id = request.program_id
        program_dict = self._manager.get(program_id)
        program = Program()
        program.program_id = str(program_dict['_id'])
        program.name = program_dict['name']
        program.xml = program_dict['xml']
        return GetProgramResponse(program)

    def handle_delete(self, request):
        program_id = request.program_id
        self._manager.delete(program_id)
        return DeleteProgramResponse()

    def handle_update(self, request):
        program = request.program
        mods = {}
        if program.name != '':
            mods['name'] = program.name
        if program.xml != '':
            mods['xml'] = program.xml
        self._manager.update(program.program_id, mods)
        return UpdateProgramResponse()

    def handle_list(self, request):
        programs = self._manager.list()
        response = ListProgramsResponse()
        for program in programs:
            prog = Program()
            prog.program_id = str(program['_id'])
            prog.name = program['name']
            prog.xml = program['xml']
            response.programs.append(prog)
        return response


def main():
    mongo_client = MongoClient()
    db = mongo_client.code_it
    program_manager = ProgramManager(db)
    server = ProgramServer(program_manager)


if __name__ == '__main__':
    rospy.init_node('program_manager')
    main()
    rospy.spin()
