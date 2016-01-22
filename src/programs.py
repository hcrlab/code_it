#!/usr/bin/env python
from bson.objectid import ObjectId
from pymongo import MongoClient
import rospy

from code_it.msg import Program
from code_it.srv import AddProgram, AddProgramResponse
from code_it.srv import DeleteProgram, DeleteProgramResponse
from code_it.srv import UpdateProgram, UpdateProgramResponse
from code_it.srv import ListPrograms, ListProgramsResponse


class ProgramManager(object):
    def __init__(self, db):
        self._db = db

    def add(self):
        result = self._db.programs.insert_one(
            {'name': 'Untitled program',
             'xml': '<xml></xml>'})
        return result.inserted_id

    def delete(self, id_str):
        result = self._db.programs.delete_one({'_id': ObjectId(id_str)})

    def update(self, id_str, mods):
        """Update a program

        id_str: The MongoDB ObjectId as a string.
        mods: A dictionary of fields to modify, e.g., {'name': 'Hello world'}
        """
        result = self._db.programs.update.update_one(
            {'_id': ObjectId(id_str)}, {'$set': mods})

    def list(self):
        return self._db.programs.find({})


class ProgramServer(object):
    def __init__(self, manager):
        self._manager = manager
        rospy.Service('code_it/add_program', AddProgram, self.handle_add)
        rospy.Service('code_it/delete_program', DeleteProgram, self.handle_delete)
        rospy.Service('code_it/update_program', UpdateProgram, self.handle_update)
        rospy.Service('code_it/list_programs', ListPrograms, self.handle_list)

    def handle_add(self, request):
        program_id = self._manager.add()
        return AddProgramResponse(program_id)
    
    def handle_delete(self, request):
        program_id = request.program_id
        self._manager.delete(program_id)
        return DeleteProgramResponse()
    
    def handle_update(self, request):
        program = request.program
        mods = {'name': program.name, 'xml': program.xml}
        self._manager.update(program.program_id, mods)
        return UpdateProgramResponse()
    
    def handle_list(self, request):
        programs = self._manager.list()
        response = ListProgramResponse()
        for program in programs:
            prog = Program()
            prog.program_id = str(response['_id'])
            prog.name = response['name']
            prog.xml = response['xml']
            response.programs.append(prog)
        return response


def main():
    client = MongoClient()
    db = client.code_it
    program_manager = ProgramManager(db)
    server = ProgramServer(program_manager) 


if __name__ == '__main__':
    rospy.init_node('program_manager')
    main()
    rospy.spin()
